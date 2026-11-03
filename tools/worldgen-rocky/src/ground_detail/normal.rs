//! Scan-normal residuals in the same source chart as the optical height field.
//! Gradients compose before normalizing; source height is not counted twice.
use super::*;

static SOIL: OnceLock<Texture<2>> = OnceLock::new();
static LITTER_NORMAL: OnceLock<Texture<2>> = OnceLock::new();
static MOSS_NORMAL: OnceLock<Texture<2>> = OnceLock::new();

fn height_gradient(mip: &Mip, x: usize, y: usize, width_m: f64) -> [f32; 2] {
    let side = mip.side;
    let at = |x: usize, y: usize| mip.pixels[y * side + x][4];
    let scale = side as f32 / (2.0 * width_m as f32);
    [
        (at((x + 1) % side, y) - at((x + side - 1) % side, y)) * scale,
        (at(x, (y + 1) % side) - at(x, (y + side - 1) % side)) * scale,
    ]
}

fn decode(bytes: &[u8], height: &Texture, width_m: f64) -> Texture<2> {
    let side = height.0[0].side;
    let expected = 8 + side * side * 3;
    let mut raw = Vec::new();
    flate2::read::GzDecoder::new(bytes)
        .take(expected as u64 + 1)
        .read_to_end(&mut raw)
        .expect("embedded ground normal gzip");
    assert_eq!(raw.len(), expected);
    assert_eq!(&raw[..4], b"GN01");
    assert_eq!(
        u32::from_le_bytes(raw[4..8].try_into().unwrap()) as usize,
        side
    );
    let mut slopes: Vec<[f32; 2]> = raw[8..]
        .as_chunks::<3>()
        .0
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let n = p.map(|c| f32::from(c) * (2.0 / 255.0) - 1.0);
            // OpenGL Y points up the image; our physical V follows its rows.
            // A folded normal has no single-valued height-graph representation.
            if n[2] > 0.0 {
                [-n[0] / n[2], n[1] / n[2]]
            } else {
                height_gradient(&height.0[0], i % side, i / side, width_m)
            }
        })
        .collect();
    // A periodic swatch must not introduce a uniform tilt into an entire world.
    let mean: [f64; 2] = std::array::from_fn(|k| {
        slopes.iter().map(|s| f64::from(s[k])).sum::<f64>() / slopes.len() as f64
    });
    for slope in &mut slopes {
        for k in 0..2 {
            slope[k] -= mean[k] as f32;
        }
    }
    let mut mips = Vec::with_capacity(height.0.len());
    for mip in &height.0 {
        let side = mip.side;
        let pixels = slopes
            .iter()
            .enumerate()
            .map(|(i, slope)| {
                let base = height_gradient(mip, i % side, i / side, width_m);
                [slope[0] - base[0], slope[1] - base[1]]
            })
            .collect();
        mips.push(Mip { side, pixels });
        if side > 1 {
            slopes = (0..side * side / 4)
                .map(|i| {
                    let x = i % (side / 2) * 2;
                    let y = i / (side / 2) * 2;
                    std::array::from_fn(|k| {
                        (slopes[y * side + x][k]
                            + slopes[y * side + x + 1][k]
                            + slopes[(y + 1) * side + x][k]
                            + slopes[(y + 1) * side + x + 1][k])
                            * 0.25
                    })
                })
                .collect();
        }
    }
    Texture(mips)
}

fn unrotate(g: [f32; 2], rotation: u32) -> [f32; 2] {
    // A sampled gradient is a covector: rotate by the inverse UV transform.
    match rotation {
        0 => g,
        1 => [g[1], -g[0]],
        2 => [-g[0], -g[1]],
        _ => [-g[1], g[0]],
    }
}

fn sample(
    texture: &Texture<2>,
    dir: [f64; 3],
    radius: f64,
    seed: u64,
    texel: f64,
    width: f64,
) -> [f32; 3] {
    let p = sample_planes(texture, dir, radius, seed, texel, width, unrotate);
    [p[1][0] + p[2][0], p[0][1] + p[2][1], p[0][0] + p[1][1]]
}

pub(crate) fn sample_residual(
    dir: [f64; 3],
    radius: f64,
    seed: u64,
    texel: f64,
    moss: f64,
) -> [f64; 3] {
    if texel >= SWATCH_WIDTH_M.max(MOSS_WIDTH_M) {
        return [0.; 3];
    }
    let q = dir.map(|v| v * radius / 8.0);
    let weight = (0.1
        + 0.75
            * crate::appearance::smooth(
                -0.25,
                0.30,
                crate::rng::perlin3(seed, 6200, q[0], q[1], q[2]),
            )
            * crate::appearance::band_weight(8.0, texel)
        + 0.375 * (1.0 - crate::appearance::band_weight(8.0, texel))) as f32;
    let a = if texel < SWATCH_WIDTH_M {
        sample(
            SOIL.get_or_init(|| {
                decode(
                    include_bytes!("../../../../assets/textures/forest-ground-04.normal.bin.gz"),
                    TEXTURE.get_or_init(Texture::decode),
                    SWATCH_WIDTH_M,
                )
            }),
            dir,
            radius,
            seed,
            texel,
            SWATCH_WIDTH_M,
        )
    } else {
        [0.; 3]
    };
    let b = if texel < LITTER_WIDTH_M {
        sample(
            LITTER_NORMAL.get_or_init(|| {
                decode(
                    include_bytes!("../../../../assets/textures/mud-litter-01.normal.bin.gz"),
                    LITTER.get_or_init(Texture::decode_litter),
                    LITTER_WIDTH_M,
                )
            }),
            dir,
            radius,
            seed ^ 0x6c6974746572,
            texel,
            LITTER_WIDTH_M,
        )
    } else {
        [0.; 3]
    };
    let m = if moss > 0. && texel < MOSS_WIDTH_M {
        sample(
            MOSS_NORMAL.get_or_init(|| {
                decode(
                    include_bytes!(
                        "../../../../assets/textures/forest-moss-litter-02.normal.bin.gz"
                    ),
                    MOSS.get_or_init(Texture::decode_moss),
                    MOSS_WIDTH_M,
                )
            }),
            dir,
            radius,
            seed ^ 0x6d6f_7373_3032,
            texel,
            MOSS_WIDTH_M,
        )
    } else {
        [0.; 3]
    };
    // Matches the half-amplitude moss scan in cover_and_moss, not its cushion.
    std::array::from_fn(|i| {
        f64::from(a[i] * (1.0 - weight) + b[i] * weight) + f64::from(m[i]) * 0.5 * moss
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn uv_rotation_transports_gradient_as_a_covector() {
        let uv = [0.17_f64, -0.29];
        let g = [0.31, -0.47];
        for rotation in 0..4 {
            let height = |p: [f64; 2]| {
                let q = match rotation {
                    0 => p,
                    1 => [-p[1], p[0]],
                    2 => [-p[0], -p[1]],
                    _ => [p[1], -p[0]],
                };
                f64::from(g[0]) * q[0] + f64::from(g[1]) * q[1]
            };
            let expected: [f64; 2] = std::array::from_fn(|i| {
                let mut a = uv;
                let mut b = uv;
                a[i] += 1e-5;
                b[i] -= 1e-5;
                (height(a) - height(b)) / 2e-5
            });
            for (a, b) in unrotate(g, rotation).into_iter().zip(expected) {
                assert!((f64::from(a) - b).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn periodic_height_is_subtracted_once_at_every_normal_mip() {
        let side = 8;
        let width = 2.;
        let pixels = (0..side * side)
            .map(|i| {
                [
                    0.,
                    0.,
                    0.,
                    1.,
                    ((i % side) as f32 * std::f32::consts::TAU / side as f32).sin() * 0.05,
                ]
            })
            .collect();
        let mut height = Texture(vec![Mip { side, pixels }]);
        while height.0.last().unwrap().side > 1 {
            let m = height.0.last().unwrap();
            let s = m.side;
            let next = s / 2;
            let pixels = (0..next * next)
                .map(|i| {
                    let x = i % next * 2;
                    let y = i / next * 2;
                    std::array::from_fn(|k| {
                        (m.pixels[y * s + x][k]
                            + m.pixels[y * s + x + 1][k]
                            + m.pixels[(y + 1) * s + x][k]
                            + m.pixels[(y + 1) * s + x + 1][k])
                            * 0.25
                    })
                })
                .collect();
            height.0.push(Mip { side: next, pixels });
        }
        let mut raw = b"GN01".to_vec();
        raw.extend((side as u32).to_le_bytes());
        let mut source = Vec::new();
        for y in 0..side {
            for x in 0..side {
                let base = height_gradient(&height.0[0], x, y, width);
                let g = [
                    base[0] + if x % 2 == 0 { 0.3 } else { -0.3 },
                    if y % 2 == 0 { 0.2 } else { -0.2 },
                ];
                let length = (1. + g[0] * g[0] + g[1] * g[1]).sqrt();
                let bytes = [-g[0] / length, g[1] / length, 1. / length]
                    .map(|v| ((v * 0.5 + 0.5) * 255.).round() as u8);
                raw.extend(bytes);
                let n = bytes.map(|v| f32::from(v) * 2. / 255. - 1.);
                source.push([-n[0] / n[2], n[1] / n[2]]);
            }
        }
        let mut packed = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        packed.write_all(&raw).unwrap();
        let normal = decode(&packed.finish().unwrap(), &height, width);
        let mean: [f32; 2] =
            std::array::from_fn(|k| source.iter().map(|s| s[k]).sum::<f32>() / source.len() as f32);
        for (mip, residual) in height.0.iter().zip(&normal.0) {
            let block = side / mip.side;
            for y in 0..mip.side {
                for x in 0..mip.side {
                    let base = height_gradient(mip, x, y, width);
                    for k in 0..2 {
                        let mut expected = 0.;
                        for yy in 0..block {
                            for xx in 0..block {
                                expected += source[(y * block + yy) * side + x * block + xx][k];
                            }
                        }
                        expected = expected / (block * block) as f32 - mean[k];
                        let got = base[k] + residual.pixels[y * mip.side + x][k];
                        assert!(
                            (got - expected).abs() < 1e-6,
                            "mip {} axis{k}: {got} vs {expected}",
                            mip.side
                        );
                    }
                }
            }
        }
        assert!(
            normal.0.last().unwrap().pixels[0]
                .iter()
                .all(|v| v.abs() < 1e-6)
        );
    }

    #[test]
    fn actual_scans_have_finite_zero_mean_residuals_and_continuous_body_phase() {
        for (bytes, height, width) in [
            (
                &include_bytes!("../../../../assets/textures/forest-ground-04.normal.bin.gz")[..],
                Texture::decode(),
                SWATCH_WIDTH_M,
            ),
            (
                &include_bytes!("../../../../assets/textures/mud-litter-01.normal.bin.gz")[..],
                Texture::decode_litter(),
                LITTER_WIDTH_M,
            ),
            (
                &include_bytes!("../../../../assets/textures/forest-moss-litter-02.normal.bin.gz")
                    [..],
                Texture::decode_moss(),
                MOSS_WIDTH_M,
            ),
        ] {
            let normal = decode(bytes, &height, width);
            assert!(
                normal
                    .0
                    .iter()
                    .flat_map(|m| &m.pixels)
                    .flatten()
                    .all(|v| v.is_finite())
            );
            assert!(
                normal.0.last().unwrap().pixels[0]
                    .iter()
                    .all(|v| v.abs() < 1e-6)
            );
            let rms = (normal.0[0]
                .pixels
                .iter()
                .flatten()
                .map(|v| v * v)
                .sum::<f32>()
                / (normal.0[0].pixels.len() * 2) as f32)
                .sqrt();
            assert!(
                rms > 0.1,
                "normal source must contain detail absent from the displacement gradient: {rms}"
            );
            for boundary in [-1.0, -0.5, 0., 0.5, 1.0] {
                let a = normal.sample_stochastic_transformed(
                    [boundary - 1e-9, 0.183],
                    0.,
                    77,
                    0,
                    width,
                    unrotate,
                );
                let b = normal.sample_stochastic_transformed(
                    [boundary + 1e-9, 0.183],
                    0.,
                    77,
                    0,
                    width,
                    unrotate,
                );
                assert!(a.into_iter().zip(b).all(|(a, b)| (a - b).abs() < 1e-5));
            }
        }
        assert_eq!(
            sample_residual([1., 0., 0.], 3_200_000., 77, 4., 1.),
            [0.; 3]
        );
    }
}
