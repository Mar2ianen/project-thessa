//! Deterministic benchmark fixtures (doc §16).
//!
//! Seven synthetic surface statistics stand in for the benchmark set until
//! real Thessa captures plug in as fixture 8:
//!
//! 1. nearly uniform regolith;
//! 2. smooth gradient;
//! 3. noisy rock;
//! 4. sharp biome/material boundary;
//! 5. checker/high-frequency adversarial pattern;
//! 6. coast/ocean boundary;
//! 7. volcanic/high-contrast terrain.
//!
//! All generators are pure functions of `(width, height, seed)` with an
//! inline xorshift64 PRNG, so fixtures are bit-identical across runs and
//! platforms without an RNG dependency.

use crate::codec::ScalarField;

fn xorshift64(state: &mut u64) -> u64 {
    // Splitmix-style avalanche keeps low bits usable for small ranges.
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn field_from_fn(width: u32, height: u32, mut f: impl FnMut(u32, u32) -> u8) -> ScalarField {
    let mut data = Vec::with_capacity(width as usize * height as usize);
    for y in 0..height {
        for x in 0..width {
            data.push(f(x, y));
        }
    }
    // Generator output always matches the declared extent.
    ScalarField::new(width, height, data).expect("fixture extent is consistent")
}

/// 1. Nearly uniform regolith: flat base with +-1 dither (saturating).
///
/// Saturation keeps base 0 dark instead of wrapping to 255.
pub fn uniform(width: u32, height: u32, base: u8) -> ScalarField {
    let mut s = 0x1234_5678_9ABC_DEF0u64;
    field_from_fn(width, height, |_, _| {
        (base as i16 + (xorshift64(&mut s) % 3) as i16 - 1).clamp(0, 255) as u8
    })
}

/// 2. Smooth horizontal gradient from 0 to 255.
pub fn gradient(width: u32, height: u32) -> ScalarField {
    field_from_fn(width, height, |x, _| {
        if width <= 1 {
            0
        } else {
            ((x as u64 * 255) / (width as u64 - 1)) as u8
        }
    })
}

/// 3. Noisy rock: uniform white noise from a seed.
pub fn noise(width: u32, height: u32, seed: u64) -> ScalarField {
    let mut s = seed | 1;
    field_from_fn(width, height, |_, _| (xorshift64(&mut s) >> 11) as u8)
}

/// 4. Sharp biome/material boundary: dark left, bright right.
pub fn sharp_boundary(width: u32, height: u32) -> ScalarField {
    field_from_fn(width, height, |x, _| if x < width / 2 { 30 } else { 220 })
}

/// 5. Adversarial high-frequency checker with `cell`-texel squares.
pub fn checker(width: u32, height: u32, cell: u32) -> ScalarField {
    let cell = cell.max(1);
    field_from_fn(width, height, |x, y| {
        if (x / cell + y / cell).is_multiple_of(2) {
            0
        } else {
            255
        }
    })
}

/// 6. Coast boundary: dark sea left of a wavy shoreline, noisy land right.
pub fn coast(width: u32, height: u32, seed: u64) -> ScalarField {
    let mut s = seed | 1;
    // Pre-generate one noise value per texel in scan order.
    let mut n = vec![0u8; width as usize * height as usize];
    for v in &mut n {
        *v = (xorshift64(&mut s) >> 11) as u8;
    }
    field_from_fn(width, height, |x, y| {
        let w = width as f64;
        let shore = w * 0.45 + 6.0 * ((y as f64) * 0.35).sin();
        let jitter = (n[y as usize * width as usize + x as usize] % 5) as f64;
        if (x as f64) < shore {
            8u8.saturating_add((jitter / 2.0) as u8)
        } else {
            let land = 120.0 + (x as f64 - shore) * 1.5 + jitter * 6.0;
            land.clamp(0.0, 255.0) as u8
        }
    })
}

/// 7. Volcanic high-contrast terrain: dark base with bright Gaussian splats.
pub fn volcanic(width: u32, height: u32, seed: u64) -> ScalarField {
    let mut s = seed | 1;
    // Deterministic splat centres and amplitudes.
    let mut splats = [(0.0f64, 0.0f64, 0.0f64, 0.0f64); 9];
    for splat in &mut splats {
        let cx = (xorshift64(&mut s) % width as u64) as f64;
        let cy = (xorshift64(&mut s) % height as u64) as f64;
        let amp = 120.0 + (xorshift64(&mut s) % 120) as f64;
        let sigma = 3.0 + (xorshift64(&mut s) % 9) as f64;
        *splat = (cx, cy, amp, sigma);
    }
    field_from_fn(width, height, |x, y| {
        let mut v = 18.0;
        for (cx, cy, amp, sigma) in splats {
            let dx = x as f64 - cx;
            let dy = y as f64 - cy;
            v += amp * (-(dx * dx + dy * dy) / (2.0 * sigma * sigma)).exp();
        }
        v.clamp(0.0, 255.0) as u8
    })
}

/// All seven fixtures at one extent, in doc §16 order.
pub fn all(width: u32, height: u32) -> Vec<(&'static str, ScalarField)> {
    vec![
        ("uniform", uniform(width, height, 128)),
        ("gradient", gradient(width, height)),
        ("noise", noise(width, height, 0xC0FFEE)),
        ("sharp", sharp_boundary(width, height)),
        ("checker", checker(width, height, 4)),
        ("coast", coast(width, height, 0x5EED)),
        ("volcanic", volcanic(width, height, 0x8A31)),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_returns_seven_fixtures_in_doc_order() {
        let all = all(16, 16);
        let names: Vec<_> = all.iter().map(|(n, _)| *n).collect();
        assert_eq!(
            names,
            [
                "uniform", "gradient", "noise", "sharp", "checker", "coast", "volcanic"
            ]
        );
        for (_, field) in &all {
            assert_eq!((field.width, field.height), (16, 16));
        }
    }

    #[test]
    fn seeded_fixtures_are_deterministic() {
        assert_eq!(noise(32, 32, 99).data, noise(32, 32, 99).data);
        assert_eq!(coast(32, 32, 99).data, coast(32, 32, 99).data);
        assert_eq!(volcanic(32, 32, 99).data, volcanic(32, 32, 99).data);
        assert_eq!(uniform(32, 32, 7).data, uniform(32, 32, 7).data);
    }

    #[test]
    fn different_seeds_differ() {
        assert_ne!(noise(32, 32, 1).data, noise(32, 32, 2).data);
        assert_ne!(coast(32, 32, 1).data, coast(32, 32, 2).data);
    }

    #[test]
    fn gradient_is_monotonic_full_range() {
        let g = gradient(64, 8);
        for y in 0..8 {
            for x in 1..64 {
                assert!(g.data[y * 64 + x] >= g.data[y * 64 + x - 1]);
            }
        }
        assert_eq!(g.data[0], 0);
        assert_eq!(g.data[63], 255);
    }

    #[test]
    fn checker_alternates_per_cell() {
        let c = checker(4, 4, 1);
        assert_eq!(
            [c.data[0], c.data[1], c.data[4], c.data[5]],
            [0, 255, 255, 0]
        );
    }

    #[test]
    fn sharp_boundary_values() {
        let s = sharp_boundary(16, 4);
        for y in 0..4 {
            for x in 0..16 {
                let expect = if x < 8 { 30 } else { 220 };
                assert_eq!(s.data[y * 16 + x], expect, "({x}, {y})");
            }
        }
    }

    #[test]
    fn uniform_stays_near_base() {
        let u = uniform(32, 32, 200);
        assert!(u.data.iter().all(|v| (198..=201).contains(v)));
        let u = uniform(32, 32, 0);
        assert!(u.data.iter().all(|v| *v <= 2));
    }

    #[test]
    fn coast_sea_is_dark_and_land_is_bright() {
        // 128 wide: shoreline sits in [51, 64]; left quarter is all sea,
        // right quarter is all land.
        let c = coast(128, 32, 5);
        let sea_max = c
            .data
            .chunks(128)
            .map(|r| r[..32].iter().max().unwrap())
            .max()
            .unwrap();
        let land_min = c
            .data
            .chunks(128)
            .map(|r| r[96..].iter().min().unwrap())
            .min()
            .unwrap();
        assert!(*sea_max <= 12, "sea {sea_max}");
        assert!(*land_min >= 120, "land {land_min}");
    }

    #[test]
    fn volcanic_has_dark_base_and_bright_splats() {
        let v = volcanic(64, 64, 11);
        let min = *v.data.iter().min().unwrap();
        let max = *v.data.iter().max().unwrap();
        assert!(min <= 30, "base {min}");
        assert!(max >= 130, "splat peak {max}");
    }

    #[test]
    fn degenerate_width_gradient_is_zero() {
        let g = gradient(1, 8);
        assert!(g.data.iter().all(|v| *v == 0));
    }
}
