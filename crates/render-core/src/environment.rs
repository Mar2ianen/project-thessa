//! Radiance environment for sky backgrounds, indirect diffuse and reflections.
//!
//! Backend-neutral: no graphics API, scene policy or physics clock. A caller
//! (the graphics/lighting policy owner) evaluates a physical sky model onto a
//! coarse equirectangular grid; renderers sample that grid by direction.
//!
//! Layout contract, shared with `render-wgpu/src/environment.wgsl`:
//!
//! - the grid covers the **full sphere** in caller-selected world axes;
//!   `+Y` is the grid's polar axis, not necessarily local planet up;
//! - `azimuth = atan2(d.z, d.x)` in `(-PI, PI]`, texel column
//!   `u = (azimuth / TAU + 0.5) * width - 0.5`;
//! - `elevation = asin(clamp(d.y, -1, 1))`, texel row
//!   `v = (elevation / PI + 0.5) * height - 0.5`;
//! - sampling is bilinear with azimuth **wrap** and elevation **clamp**.
//!
//! Radiance is stored in the surface-lighting scale (the same "lux" scale the
//! direct light colours use, per steradian), so a uniform field of
//! `ambient_lux / PI` is exactly the legacy isotropic environment term
//! `ambient_lux / PI`; [`hemispherical_irradiance`] normalizes its quadrature
//! so that identity holds for a uniform field up to floating-point rounding.
#![forbid(unsafe_code)]

use std::f32::consts::PI;

/// Azimuth bins accepted by the contract and by the WGSL sampler.
pub const MAX_WIDTH: u32 = 256;
/// Elevation bins accepted by the contract and by the WGSL sampler.
pub const MAX_HEIGHT: u32 = 128;
/// Upper bound on texels, matching the validation in [`EnvironmentRadiance::new`].
pub const MAX_TEXELS: usize = (MAX_WIDTH * MAX_HEIGHT) as usize;

/// A validated radiance environment grid.
#[derive(Debug, Clone, PartialEq)]
pub struct EnvironmentRadiance {
    width: u32,
    height: u32,
    texels: Vec<[f32; 3]>,
    ambient_lux: [f32; 3],
    diffuse_coefficients: [[f32; 3]; 9],
}

impl EnvironmentRadiance {
    /// Validate and integrate a grid. Rejects degenerate dimensions,
    /// non-finite or negative radiance, and a texel count that does not match
    /// the dimensions, rather than rendering a partially valid field.
    pub fn new(width: u32, height: u32, texels: Vec<[f32; 3]>) -> Option<Self> {
        Self::with_up(width, height, texels, [0.0, 1.0, 0.0])
    }

    /// Integrate the ambient about `up` without rotating the grid axes. A
    /// planet renderer supplies the camera's radial up, not the body's +Y.
    pub fn with_up(width: u32, height: u32, texels: Vec<[f32; 3]>, up: [f32; 3]) -> Option<Self> {
        if !(2..=MAX_WIDTH).contains(&width) || !(2..=MAX_HEIGHT).contains(&height) {
            return None;
        }
        if texels.len() != (width * height) as usize {
            return None;
        }
        if texels
            .iter()
            .any(|texel| texel.iter().any(|c| !c.is_finite() || *c < 0.0))
        {
            return None;
        }
        let ambient_lux = irradiance(width, height, &texels, up)?;
        let diffuse_coefficients = diffuse_coefficients(width, height, &texels)?;
        Some(Self {
            width,
            height,
            texels,
            ambient_lux,
            diffuse_coefficients,
        })
    }

    /// Three-band Lambert-convolved radiance SH in the grid frame.
    /// Computed only when the source field is constructed, never per pixel.
    pub fn diffuse_coefficients(&self) -> &[[f32; 3]; 9] {
        &self.diffuse_coefficients
    }
    pub fn diffuse_irradiance(&self, normal: [f32; 3]) -> Option<[f32; 3]> {
        let n = glam::DVec3::from_array(normal.map(f64::from)).try_normalize()?;
        if !n.is_finite() {
            return None;
        }
        let basis = diffuse_basis(n);
        Some(std::array::from_fn(|c| {
            self.diffuse_coefficients
                .iter()
                .zip(basis)
                .map(|(v, b)| f64::from(v[c]) * b)
                .sum::<f64>()
                .max(0.0) as f32
        }))
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn texels(&self) -> &[[f32; 3]] {
        &self.texels
    }

    /// Cos-weighted irradiance about the supplied up direction, in lux.
    pub fn ambient_lux(&self) -> [f32; 3] {
        self.ambient_lux
    }

    /// Device storage for the grid, excluding the small metadata uniform.
    pub fn storage_bytes(&self) -> u64 {
        // Renderers store vec4<f32> texels; keep the reported figure on that
        // transport rather than the packed CPU layout.
        self.texels.len() as u64 * 16
    }

    /// CPU mirror of the WGSL sampler, used by oracles and diagnostics.
    pub fn sample(&self, direction: [f32; 3]) -> [f32; 3] {
        let length_squared = direction.iter().map(|c| c * c).sum::<f32>();
        let direction = if length_squared.is_finite() && length_squared > 0.0 {
            direction.map(|c| c / length_squared.sqrt())
        } else {
            [0.0, 1.0, 0.0]
        };
        let elevation = direction[1].clamp(-1.0, 1.0).asin();
        let azimuth = direction[2].atan2(direction[0]);
        let width = self.width as f32;
        let height = self.height as f32;
        let u = (azimuth / (2.0 * PI) + 0.5) * width - 0.5;
        let v = ((elevation / PI + 0.5) * height - 0.5).clamp(0.0, height - 1.0);
        let x0 = u.floor();
        let y0 = v.floor();
        let tx = u - x0;
        let ty = v - y0;
        let column = |x: f32| x.rem_euclid(width).clamp(0.0, width - 1.0) as usize;
        let row = |y: f32| y.clamp(0.0, height - 1.0) as usize;
        let (x0, x1) = (column(x0), column(x0 + 1.0));
        let (y0, y1) = (row(y0), row(y0 + 1.0));
        let width = self.width as usize;
        let texel = |x: usize, y: usize| self.texels[y * width + x];
        let lerp = |a: f32, b: f32, t: f32| a + t * (b - a);
        let mut out = [0.0_f32; 3];
        for (channel, value) in out.iter_mut().enumerate() {
            let top = lerp(texel(x0, y0)[channel], texel(x1, y0)[channel], tx);
            let bottom = lerp(texel(x0, y1)[channel], texel(x1, y1)[channel], tx);
            *value = lerp(top, bottom, ty);
        }
        out
    }
}

// Real orthonormal SH, l<=2. Lambert convolution factors are pi, 2pi/3,
// pi/4 (Ramamoorthi & Hanrahan, SIGGRAPH 2001). Body axes never rotate here.
fn diffuse_basis(n: glam::DVec3) -> [f64; 9] {
    let [x, y, z] = n.to_array();
    [
        0.28209479177387814,
        0.4886025119029199 * y,
        0.4886025119029199 * z,
        0.4886025119029199 * x,
        1.0925484305920792 * x * y,
        1.0925484305920792 * y * z,
        0.31539156525252005 * (3.0 * z * z - 1.0),
        1.0925484305920792 * x * z,
        0.5462742152960396 * (x * x - y * y),
    ]
}
fn diffuse_coefficients(width: u32, height: u32, texels: &[[f32; 3]]) -> Option<[[f32; 3]; 9]> {
    let mut mean = [0.0; 3];
    let mut cells = Vec::with_capacity(texels.len());
    for y in 0..height {
        let lo = (f64::from(y) / f64::from(height) - 0.5) * std::f64::consts::PI;
        let hi = (f64::from(y + 1) / f64::from(height) - 0.5) * std::f64::consts::PI;
        let elevation = (lo + hi) * 0.5;
        let area = std::f64::consts::TAU / f64::from(width) * (hi.sin() - lo.sin());
        for x in 0..width {
            let az = ((f64::from(x) + 0.5) / f64::from(width) - 0.5) * std::f64::consts::TAU;
            let d = glam::DVec3::new(
                elevation.cos() * az.cos(),
                elevation.sin(),
                elevation.cos() * az.sin(),
            );
            let value = texels[(y * width + x) as usize].map(f64::from);
            for c in 0..3 {
                mean[c] += value[c] * area / (4.0 * std::f64::consts::PI);
            }
            cells.push((diffuse_basis(d), area, value));
        }
    }
    let mut coefficients = [[0.0_f64; 3]; 9];
    for c in 0..3 {
        coefficients[0][c] = mean[c] * (4.0 * std::f64::consts::PI).sqrt() * std::f64::consts::PI;
    }
    for (basis, area, value) in cells {
        for i in 1..9 {
            let convolution = if i < 4 {
                2.0 * std::f64::consts::PI / 3.0
            } else {
                std::f64::consts::PI / 4.0
            };
            for c in 0..3 {
                coefficients[i][c] += (value[c] - mean[c]) * basis[i] * area * convolution;
            }
        }
    }
    let result = coefficients.map(|v| v.map(|c| c as f32));
    result
        .iter()
        .flatten()
        .all(|v| v.is_finite())
        .then_some(result)
}

/// Cosine-weighted +Y hemisphere irradiance, normalized to `PI` for a uniform
/// field. Directional fields use texel-centre quadrature, not a full convolution.
pub fn hemispherical_irradiance(width: u32, height: u32, texels: &[[f32; 3]]) -> Option<[f32; 3]> {
    irradiance(width, height, texels, [0.0, 1.0, 0.0])
}

fn irradiance(width: u32, height: u32, texels: &[[f32; 3]], up: [f32; 3]) -> Option<[f32; 3]> {
    if !(2..=MAX_WIDTH).contains(&width) || !(2..=MAX_HEIGHT).contains(&height) {
        return None;
    }
    if texels.len() != (width * height) as usize {
        return None;
    }
    let up = glam::DVec3::from_array(up.map(f64::from)).try_normalize()?;
    if !up.is_finite() {
        return None;
    }
    let rows = f64::from(height);
    let columns = f64::from(width);
    let mut total = 0.0_f64;
    let mut irradiance = [0.0_f64; 3];
    for row in 0..height {
        let low = (f64::from(row) / rows - 0.5) * std::f64::consts::PI;
        let high = (f64::from(row + 1) / rows - 0.5) * std::f64::consts::PI;
        let elevation = (low + high) * 0.5;
        let solid_angle = std::f64::consts::TAU / columns * (high.sin() - low.sin());
        for column in 0..width {
            let azimuth = ((f64::from(column) + 0.5) / columns - 0.5) * std::f64::consts::TAU;
            let direction = glam::DVec3::new(
                elevation.cos() * azimuth.cos(),
                elevation.sin(),
                elevation.cos() * azimuth.sin(),
            );
            let weight = solid_angle * direction.dot(up).max(0.0);
            total += weight;
            let texel = texels[(row * width + column) as usize];
            for (channel, value) in texel.iter().enumerate() {
                if !value.is_finite() || *value < 0.0 {
                    return None;
                }
                irradiance[channel] += weight * f64::from(*value);
            }
        }
    }
    if total <= 0.0 {
        return None;
    }
    let scale = std::f64::consts::PI / total;
    let mut out = [0.0_f32; 3];
    for (channel, value) in irradiance.iter().enumerate() {
        let value = *value * scale;
        if !value.is_finite() || value > f64::from(f32::MAX) {
            return None;
        }
        out[channel] = value as f32;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diffuse_irradiance_matches_uniform_and_rotated_half_sky_oracles() {
        for resolution in [(8, 4), (64, 32)] {
            let f = EnvironmentRadiance::new(
                resolution.0,
                resolution.1,
                vec![[2.0; 3]; (resolution.0 * resolution.1) as usize],
            )
            .unwrap();
            for n in [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.3, -0.4, 0.5],
            ] {
                for v in f.diffuse_irradiance(n).unwrap() {
                    assert!((v - 2.0 * PI).abs() < 0.00001);
                }
            }
        }
        for up in [glam::DVec3::Y, glam::DVec3::new(0.7, 0.2, -0.6).normalize()] {
            let mut texels = Vec::new();
            for y in 0..64 {
                for x in 0..128 {
                    let e = ((y as f64 + 0.5) / 64.0 - 0.5) * std::f64::consts::PI;
                    let a = ((x as f64 + 0.5) / 128.0 - 0.5) * std::f64::consts::TAU;
                    let d = glam::DVec3::new(e.cos() * a.cos(), e.sin(), e.cos() * a.sin());
                    texels.push([if d.dot(up) > 0.0 { 1.0 } else { 0.0 }; 3]);
                }
            }
            let field = EnvironmentRadiance::new(128, 64, texels).unwrap();
            for n in [
                glam::DVec3::X,
                glam::DVec3::Y,
                glam::DVec3::Z,
                -up,
                up,
                glam::DVec3::new(0.4, -0.5, 0.3).normalize(),
            ] {
                let expected = std::f64::consts::PI * (1.0 + n.dot(up)) * 0.5;
                for value in field.diffuse_irradiance(n.as_vec3().to_array()).unwrap() {
                    assert!(
                        (f64::from(value) - expected).abs() < 0.004 * std::f64::consts::PI,
                        "half sky {up:?} n={n:?}: {value} vs {expected}"
                    );
                }
            }
        }
    }

    fn uniform(width: u32, height: u32, radiance: f32) -> EnvironmentRadiance {
        EnvironmentRadiance::new(
            width,
            height,
            vec![[radiance; 3]; (width * height) as usize],
        )
        .unwrap()
    }

    /// A uniform field samples to its own radiance in every direction,
    /// including the poles, the azimuth seam and a degenerate direction.
    #[test]
    fn uniform_field_samples_exactly_everywhere() {
        let field = uniform(8, 4, 0.75);
        let directions = [
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [0.577, 0.577, 0.577],
            [0.0, 0.0, 0.0],
        ];
        for direction in directions {
            assert_eq!(field.sample(direction), [0.75; 3], "{direction:?}");
        }
    }

    /// The legacy isotropic term is `ambient_lux / PI`; a field whose texels
    /// are exactly that value must report `ambient_lux` back, so switching a
    /// scene from the isotropic term to a uniform field is a no-op.
    #[test]
    fn uniform_field_reports_its_own_ambient() {
        for (width, height) in [(2, 2), (8, 4), (16, 8), (32, 16), (7, 5), (256, 128)] {
            let radiance = 1.25_f32 / PI;
            let field = uniform(width, height, radiance);
            for channel in 0..3 {
                assert!(
                    (field.ambient_lux()[channel] - 1.25).abs() < 1e-5,
                    "{width}x{height} channel {channel}: {}",
                    field.ambient_lux()[channel]
                );
            }
        }
    }

    #[test]
    fn hemisphere_uses_local_radial_up_not_grid_polar_axis() {
        let width = 32;
        let height = 16;
        let texels = (0..height)
            .flat_map(|row| {
                (0..width).map(move |_| {
                    if row >= height / 2 {
                        [1.0; 3]
                    } else {
                        [0.0; 3]
                    }
                })
            })
            .collect::<Vec<_>>();
        let up =
            EnvironmentRadiance::with_up(width, height, texels.clone(), [0.0, 1.0, 0.0]).unwrap();
        let down =
            EnvironmentRadiance::with_up(width, height, texels.clone(), [0.0, -1.0, 0.0]).unwrap();
        let side = EnvironmentRadiance::with_up(width, height, texels, [1.0, 0.0, 0.0]).unwrap();
        assert!((up.ambient_lux()[0] - PI).abs() < 1e-6);
        assert_eq!(down.ambient_lux(), [0.0; 3]);
        assert!((side.ambient_lux()[0] - PI / 2.0).abs() < 1e-6);
        assert!(EnvironmentRadiance::with_up(2, 2, vec![[1.0; 3]; 4], [0.0; 3]).is_none());
    }

    /// Directional fields are sampled by direction, not by azimuth alone.
    #[test]
    fn sampling_follows_the_sampled_direction() {
        let mut texels = vec![[0.0_f32; 3]; 4 * 2];
        // Column 0 spans azimuth (-PI, -PI/2): the -X/-Z quadrant.
        for row in 0..2 {
            texels[row * 4] = [1.0, 0.0, 0.0];
        }
        let field = EnvironmentRadiance::new(4, 2, texels).unwrap();
        let inside = std::f32::consts::FRAC_1_SQRT_2;
        assert_eq!(field.sample([-inside, 0.0, -inside]), [1.0, 0.0, 0.0]);
        // The azimuth seam at -X sits exactly between the last and first
        // column, so it must interpolate rather than pick one side.
        assert_eq!(field.sample([-1.0, 0.0, 0.0])[0], 0.5);
        assert_eq!(field.sample([1.0, 0.0, 0.0])[0], 0.0);
        assert_eq!(field.sample([0.0, 1.0, 0.0])[0], 0.0);
    }

    /// Metadata must describe the grid rather than a partially valid field.
    #[test]
    fn invalid_grids_are_rejected() {
        assert!(EnvironmentRadiance::new(1, 4, vec![[1.0; 3]; 4]).is_none());
        assert!(EnvironmentRadiance::new(8, 4, vec![[1.0; 3]; 31]).is_none());
        assert!(EnvironmentRadiance::new(MAX_WIDTH + 1, 4, Vec::new()).is_none());
        assert!(EnvironmentRadiance::new(4, MAX_HEIGHT + 1, Vec::new()).is_none());
        let mut negative = vec![[1.0_f32; 3]; 8];
        negative[3] = [1.0, -0.5, 1.0];
        assert!(EnvironmentRadiance::new(2, 4, negative).is_none());
        let mut nan = vec![[1.0_f32; 3]; 8];
        nan[0] = [f32::NAN; 3];
        assert!(EnvironmentRadiance::new(2, 4, nan).is_none());
        // Below-horizon texels carry radiance but no upper-hemisphere weight.
        let low = EnvironmentRadiance::new(4, 2, vec![[5.0_f32; 3]; 8]).unwrap();
        assert!((low.ambient_lux()[0] - 5.0 * PI).abs() < 1e-4);
    }

    /// Texel centres round-trip through the documented azimuth/elevation map.
    #[test]
    fn texel_centres_map_to_their_own_texels() {
        let mut texels = vec![[0.0_f32; 3]; 16 * 8];
        for (index, texel) in texels.iter_mut().enumerate() {
            *texel = [(index as f32) / 255.0; 3];
        }
        let field = EnvironmentRadiance::new(16, 8, texels.clone()).unwrap();
        for row in 0..8u32 {
            for column in 0..16u32 {
                let azimuth = ((f64::from(column) + 0.5) / 16.0 - 0.5) * std::f64::consts::TAU;
                let elevation = ((f64::from(row) + 0.5) / 8.0 - 0.5) * std::f64::consts::PI;
                let direction = [
                    (elevation.cos() * azimuth.cos()) as f32,
                    elevation.sin() as f32,
                    (elevation.cos() * azimuth.sin()) as f32,
                ];
                let sampled = field.sample(direction);
                let expected = texels[(row * 16 + column) as usize];
                for channel in 0..3 {
                    assert!(
                        (sampled[channel] - expected[channel]).abs() < 1e-4,
                        "row {row} column {column} channel {channel}"
                    );
                }
            }
        }
    }
}
