//! Independently implemented linear finite-depth spectral sea (Tessendorf,
//! *Simulating Ocean Water*, 2001). SI inputs; no backend or simulation owner.
//! Fourier coefficients use a spatial sum convention, not FFT normalization.
use std::f64::consts::{PI, TAU};

pub const PERIODS_M: [f64; 3] = [2048.0, 256.0, 32.0];
pub const PLANES: usize = 3;
pub const LAYERS: usize = PERIODS_M.len() * PLANES;
pub const REFERENCE_SIZE: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeaState {
    pub gravity_m_s2: f64,
    pub depth_m: f64,
    pub wind_speed_m_s: f64,
    pub wind_direction_rad: f64,
    /// Target ensemble significant height of each planar reference field.
    pub significant_height_m: f64,
    pub seed: u64,
}

impl SeaState {
    pub fn valid(self) -> bool {
        (0.001..=1000.0).contains(&self.gravity_m_s2)
            && (0.1..=100_000.0).contains(&self.depth_m)
            && (0.0..=100.0).contains(&self.wind_speed_m_s)
            && self.wind_direction_rad.is_finite()
            && (0.0..=20.0).contains(&self.significant_height_m)
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Mode {
    pub initial: [f64; 2],
    pub opposite: [f64; 2],
    pub wave_number: [f64; 2],
    pub omega: f64,
}

impl Mode {
    pub fn phase(self, time_s: f64) -> f64 {
        if self.omega == 0.0 {
            0.0
        } else {
            time_s.rem_euclid(TAU / self.omega) * self.omega
        }
    }

    pub fn at(self, time_s: f64) -> [f64; 2] {
        let (s, c) = self.phase(time_s).sin_cos();
        [
            (self.initial[0] + self.opposite[0]) * c - (self.initial[1] + self.opposite[1]) * s,
            (self.initial[0] - self.opposite[0]) * s + (self.initial[1] - self.opposite[1]) * c,
        ]
    }
}

#[derive(Debug)]
pub struct Spectrum {
    pub size: usize,
    pub modes: Vec<Mode>,
    /// Per plane: unresolved mean squared slope after frequency truncation.
    pub unresolved_slope_variance: [f64; PLANES],
    /// Conservative absolute height envelope, including omitted frequencies.
    pub height_bound_m: f64,
}

fn signed(i: usize, n: usize) -> f64 {
    if i < n / 2 {
        i as f64
    } else {
        i as f64 - n as f64
    }
}

fn random(state: &mut u64) -> f64 {
    *state = state.wrapping_add(0x9e3779b97f4a7c15);
    let mut x = *state;
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    x ^= x >> 31;
    ((x >> 11) as f64 + 0.5) / (1u64 << 53) as f64
}

impl Spectrum {
    pub fn new(state: SeaState, size: usize) -> Option<Self> {
        if !state.valid() || !size.is_power_of_two() || !(8..=REFERENCE_SIZE).contains(&size) {
            return None;
        }
        let n = REFERENCE_SIZE;
        let mut full = vec![Mode::default(); n * n * LAYERS];
        let mut rng = state.seed;
        let wind = [
            state.wind_direction_rad.cos(),
            state.wind_direction_rad.sin(),
        ];
        for plane in 0..PLANES {
            let mut power = 0.0;
            for (band, &period) in PERIODS_M.iter().enumerate() {
                let lower = if band == 0 {
                    0.0
                } else {
                    PI * n as f64 / PERIODS_M[band - 1]
                };
                for y in 0..n {
                    for x in 0..n {
                        let k = [signed(x, n) * TAU / period, signed(y, n) * TAU / period];
                        let length = k[0].hypot(k[1]);
                        if length <= lower
                            || length >= PI * n as f64 / period
                            || x == n / 2
                            || y == n / 2
                            || state.wind_speed_m_s == 0.0
                        {
                            continue;
                        }
                        let wind_length = state.wind_speed_m_s.powi(2) / state.gravity_m_s2;
                        let alignment = (k[0] * wind[0] + k[1] * wind[1]) / length;
                        // Phillips shape; physical capillary/gravity crossover
                        // regularizes the small-scale tail (water: sigma/rho).
                        let capillary_length2 = 0.074 / (1025.0 * state.gravity_m_s2);
                        let shape = (-1.0 / (length * wind_length).powi(2)
                            - length.powi(2) * capillary_length2)
                            .exp()
                            * alignment.powi(2)
                            / length.powi(4)
                            * (TAU / period).powi(2);
                        let radius = (-2.0 * random(&mut rng).ln()).sqrt() * shape.sqrt();
                        let angle = random(&mut rng) * TAU;
                        let initial = [radius * angle.cos(), radius * angle.sin()];
                        power += 2.0 * (initial[0].powi(2) + initial[1].powi(2));
                        full[(plane * 3 + band) * n * n + y * n + x] = Mode {
                            initial,
                            opposite: [0.0; 2],
                            wave_number: k,
                            omega: (state.gravity_m_s2 * length * (length * state.depth_m).tanh())
                                .sqrt(),
                        };
                    }
                }
            }
            let scale = if power > 0.0 {
                state.significant_height_m / (4.0 * power.sqrt())
            } else {
                0.0
            };
            for mode in &mut full[plane * 3 * n * n..(plane + 1) * 3 * n * n] {
                mode.initial[0] *= scale;
                mode.initial[1] *= scale;
            }
        }
        let mut modes = Vec::with_capacity(size * size * LAYERS);
        let mut omitted = [0.0; PLANES];
        let mut bound = [0.0; PLANES];
        for layer in 0..LAYERS {
            for y in 0..n {
                for x in 0..n {
                    let mode = full[layer * n * n + y * n + x];
                    bound[layer / 3] += 2.0 * mode.initial[0].hypot(mode.initial[1]);
                    if signed(x, n).abs() >= (size / 2) as f64
                        || signed(y, n).abs() >= (size / 2) as f64
                    {
                        omitted[layer / 3] += 2.0
                            * (mode.initial[0].powi(2) + mode.initial[1].powi(2))
                            * (mode.wave_number[0].powi(2) + mode.wave_number[1].powi(2));
                    }
                }
            }
            for y in 0..size {
                for x in 0..size {
                    let sx = signed(x, size).rem_euclid(n as f64) as usize;
                    let sy = signed(y, size).rem_euclid(n as f64) as usize;
                    let mut mode = full[layer * n * n + sy * n + sx];
                    mode.opposite = full[layer * n * n + ((n - sy) % n) * n + (n - sx) % n].initial;
                    if x == size / 2 || y == size / 2 {
                        mode = Mode::default();
                    }
                    modes.push(mode);
                }
            }
        }
        Some(Self {
            size,
            modes,
            unresolved_slope_variance: omitted,
            height_bound_m: bound.into_iter().map(|v| v * v).sum::<f64>().sqrt(),
        })
    }

    /// Independent direct Fourier-sum oracle, intentionally not a production
    /// spatial evaluator. Returns height and two spatial derivatives in SI.
    pub fn sample_reference(&self, layer: usize, uv: [f64; 2], time_s: f64) -> [f64; 3] {
        let mut out = [0.0; 3];
        for y in 0..self.size {
            for x in 0..self.size {
                let m = self.modes[layer * self.size * self.size + y * self.size + x];
                let value = m.at(time_s);
                let angle = TAU * (signed(x, self.size) * uv[0] + signed(y, self.size) * uv[1]);
                let (s, c) = angle.sin_cos();
                out[0] += value[0] * c - value[1] * s;
                let derivative = -value[0] * s - value[1] * c;
                out[1] += derivative * m.wave_number[0];
                out[2] += derivative * m.wave_number[1];
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sea() -> SeaState {
        SeaState {
            gravity_m_s2: 7.0,
            depth_m: 100.0,
            wind_speed_m_s: 10.0,
            wind_direction_rad: 0.4,
            significant_height_m: 2.0,
            seed: 7,
        }
    }
    #[test]
    fn conjugate_pairs_zero_mode_and_periodic_derivatives() {
        let s = Spectrum::new(sea(), 16).unwrap();
        for layer in 0..LAYERS {
            for y in 0..16 {
                for x in 0..16 {
                    let a = s.modes[layer * 256 + y * 16 + x].at(123.5);
                    let b = s.modes[layer * 256 + ((16 - y) % 16) * 16 + (16 - x) % 16].at(123.5);
                    assert!((a[0] - b[0]).abs() < 1e-12 && (a[1] + b[1]).abs() < 1e-12);
                }
            }
            let p = s.sample_reference(layer, [0.23, 0.37], 123.5);
            let q = s.sample_reference(layer, [1.23, -0.63], 123.5);
            assert!(p.iter().zip(q).all(|(a, b)| (a - b).abs() < 1e-12));
        }
    }
    #[test]
    fn calm_disabled_frequency_truncation_and_dispersion() {
        let mut state = sea();
        state.significant_height_m = 0.0;
        let calm = Spectrum::new(state, 8).unwrap();
        assert_eq!(calm.sample_reference(0, [0.2, 0.3], 1e9), [0.0; 3]);
        state = sea();
        let low = Spectrum::new(state, 32).unwrap();
        let high = Spectrum::new(state, 128).unwrap();
        assert!(low.unresolved_slope_variance.iter().all(|&v| v > 0.0));
        assert_eq!(high.unresolved_slope_variance, [0.0; 3]);
        for m in &high.modes {
            let k = m.wave_number[0].hypot(m.wave_number[1]);
            assert!(
                (m.omega.powi(2) - state.gravity_m_s2 * k * (k * state.depth_m).tanh()).abs()
                    < 1e-10
            );
        }
        state.gravity_m_s2 = f64::NAN;
        assert!(Spectrum::new(state, 32).is_none());
    }

    #[test]
    fn finite_depth_long_wave_limit_and_local_gravity_scaling() {
        let mut state = sea();
        state.depth_m = 0.1;
        let shallow = Spectrum::new(state, 32).unwrap();
        let m = shallow.modes[1];
        let k = m.wave_number[0].hypot(m.wave_number[1]);
        assert!((m.omega / k - (state.gravity_m_s2 * state.depth_m).sqrt()).abs() < 1e-7);
        state.depth_m = 100_000.0;
        let deep = Spectrum::new(state, 32).unwrap();
        let m = deep.modes[1];
        assert!((m.omega.powi(2) - state.gravity_m_s2 * k).abs() < 1e-12);
        state.gravity_m_s2 *= 0.25;
        let low_g = Spectrum::new(state, 32).unwrap();
        assert!((low_g.modes[1].omega / m.omega - 0.5).abs() < 1e-12);
        for mode in &deep.modes {
            let [real, imag] = mode.at(1e9);
            assert!(real.is_finite() && imag.is_finite());
            let mut direct = *mode;
            direct.omega = 0.0;
            let angle = mode.omega * 1e9;
            let (s, c) = angle.sin_cos();
            let expected = (direct.initial[0] + direct.opposite[0]) * c
                - (direct.initial[1] + direct.opposite[1]) * s;
            assert!((real - expected).abs() < 1e-6, "long-time phase bound");
        }
    }
}
