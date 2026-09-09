//! Deterministic seeded hashing / value noise in physical units.
//!
//! No RNG state is ever carried: every sample is a pure function of
//! (seed, octave/channel, integer lattice coords). Same inputs => same bits.

/// SplitMix64-style stateless hash to [0,1).
pub fn hash01(seed: u64, channel: u32, ix: i64, iy: i64) -> f64 {
    let mut h = seed
        .wrapping_add(u64::from(channel).wrapping_mul(0x9E37_79B9_7F4A_7C15))
        .wrapping_add((ix as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9))
        .wrapping_add((iy as u64).wrapping_mul(0x94D0_49BB_1331_11EB));
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^= h >> 31;
    (h as f64) / (u64::MAX as f64)
}

/// Signed variant in [-1,1].
pub fn hash11(seed: u64, channel: u32, ix: i64, iy: i64) -> f64 {
    hash01(seed, channel, ix, iy) * 2.0 - 1.0
}

/// Smooth value noise over a 2D lattice. Inputs are in *feature-local* units
/// (callers scale physical metres to lattice steps first).
pub fn value_noise(seed: u64, channel: u32, x: f64, y: f64) -> f64 {
    let ix = x.floor() as i64;
    let iy = y.floor() as i64;
    let fx = x - ix as f64;
    let fy = y - iy as f64;
    let sx = fx * fx * (3.0 - 2.0 * fx);
    let sy = fy * fy * (3.0 - 2.0 * fy);
    let a = hash11(seed, channel, ix, iy);
    let b = hash11(seed, channel, ix + 1, iy);
    let c = hash11(seed, channel, ix, iy + 1);
    let d = hash11(seed, channel, ix + 1, iy + 1);
    a + (b - a) * sx + (c - a) * sy + (a - b - c + d) * sx * sy
}

/// Fractal sum over `octaves`, pure function of inputs.
pub fn fbm(seed: u64, channel: u32, x: f64, y: f64, octaves: u32) -> f64 {
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut sum = 0.0;
    let mut norm = 0.0;
    for octave in 0..octaves {
        sum += amplitude * value_noise(seed, channel + octave * 7919, x * frequency, y * frequency);
        norm += amplitude;
        amplitude *= 0.5;
        frequency *= 2.03;
    }
    sum / norm.max(f64::MIN_POSITIVE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hash_is_deterministic_and_spread() {
        let a = hash01(7, 0, 3, -5);
        assert_eq!(a, hash01(7, 0, 3, -5));
        assert!((0.0..1.0).contains(&a));
        assert!((hash01(8, 0, 3, -5) - a).abs() > 1e-12);
    }

    #[test]
    fn fbm_stays_bounded_and_deterministic() {
        let a = fbm(7, 11, 1.7, -2.3, 4);
        assert_eq!(a, fbm(7, 11, 1.7, -2.3, 4));
        assert!(a.abs() <= 1.0 + 1e-12);
        assert!(fbm(7, 11, 1.7, -2.3, 4).is_finite());
    }
}
