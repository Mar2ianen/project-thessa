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

/// Stateless 3D lattice hash to [0,1).
pub fn hash3_01(seed: u64, channel: u32, ix: i64, iy: i64, iz: i64) -> f64 {
    let mut h = seed
        .wrapping_add(u64::from(channel).wrapping_mul(0x9E37_79B9_7F4A_7C15))
        .wrapping_add((ix as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9))
        .wrapping_add((iy as u64).wrapping_mul(0x94D0_49BB_1331_11EB))
        .wrapping_add((iz as u64).wrapping_mul(0x368E_9E5A_1A74_265D));
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^= h >> 31;
    (h as f64) / (u64::MAX as f64)
}

/// Signed 3D lattice hash in [-1,1].
pub fn hash3_11(seed: u64, channel: u32, ix: i64, iy: i64, iz: i64) -> f64 {
    hash3_01(seed, channel, ix, iy, iz) * 2.0 - 1.0
}

fn smooth(t: f64) -> f64 {
    t * t * (3.0 - 2.0 * t)
}

/// 3D value noise, pure function of inputs.
pub fn value_noise3(seed: u64, channel: u32, x: f64, y: f64, z: f64) -> f64 {
    let ix = x.floor() as i64;
    let iy = y.floor() as i64;
    let iz = z.floor() as i64;
    let fx = smooth(x - ix as f64);
    let fy = smooth(y - iy as f64);
    let fz = smooth(z - iz as f64);
    let c000 = hash3_11(seed, channel, ix, iy, iz);
    let c100 = hash3_11(seed, channel, ix + 1, iy, iz);
    let c010 = hash3_11(seed, channel, ix, iy + 1, iz);
    let c110 = hash3_11(seed, channel, ix + 1, iy + 1, iz);
    let c001 = hash3_11(seed, channel, ix, iy, iz + 1);
    let c101 = hash3_11(seed, channel, ix + 1, iy, iz + 1);
    let c011 = hash3_11(seed, channel, ix, iy + 1, iz + 1);
    let c111 = hash3_11(seed, channel, ix + 1, iy + 1, iz + 1);
    let x00 = c000 + (c100 - c000) * fx;
    let x10 = c010 + (c110 - c010) * fx;
    let x01 = c001 + (c101 - c001) * fx;
    let x11 = c011 + (c111 - c011) * fx;
    let y0 = x00 + (x10 - x00) * fy;
    let y1 = x01 + (x11 - x01) * fy;
    y0 + (y1 - y0) * fz
}

/// 3D fBm, normalized to roughly [-1,1].
pub fn fbm3(seed: u64, channel: u32, x: f64, y: f64, z: f64, octaves: u32) -> f64 {
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut sum = 0.0;
    let mut norm = 0.0;
    for octave in 0..octaves {
        sum += amplitude
            * value_noise3(
                seed,
                channel + octave * 7919,
                x * frequency,
                y * frequency,
                z * frequency,
            );
        norm += amplitude;
        amplitude *= 0.5;
        frequency *= 2.03;
    }
    sum / norm.max(f64::MIN_POSITIVE)
}

#[cfg(test)]
mod tests3d {
    use super::{fbm3, hash3_01, value_noise3};

    #[test]
    fn hash3_deterministic_bounded() {
        let a = hash3_01(7, 3, 1, -2, 5);
        assert_eq!(a, hash3_01(7, 3, 1, -2, 5));
        assert!((0.0..1.0).contains(&a));
        assert!((hash3_01(8, 3, 1, -2, 5) - a).abs() > 1e-12);
    }

    #[test]
    fn noise3_continuous_and_bounded() {
        let a = value_noise3(7, 3, 1.2, -0.4, 2.9);
        let b = value_noise3(7, 3, 1.2 + 1e-6, -0.4, 2.9);
        assert!((a - b).abs() < 1e-3, "value noise must be continuous");
        assert!(fbm3(7, 3, 1.2, -0.4, 2.9, 4).abs() <= 1.0 + 1e-12);
    }
}
