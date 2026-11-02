//! Cached solar transport from the existing optical model. Coordinates follow
//! atmosphere chord distance and radial height, concentrating samples near the
//! horizon. No graphics API, light authoring, simulation state or fitted colour.
use crate::{AtmosphereOptics, scattering::unobstructed_transmittance};
use glam::DVec3;

#[derive(Debug)]
pub struct TransmittanceLut {
    optics: AtmosphereOptics,
    width: u32,
    height: u32,
    steps: u32,
    texels: Vec<[f32; 3]>,
}
impl TransmittanceLut {
    pub fn build(
        optics: &AtmosphereOptics,
        width: u32,
        height: u32,
        steps: u32,
    ) -> Result<Self, &'static str> {
        optics.validate().map_err(|_| "invalid atmosphere optics")?;
        if !(2..=512).contains(&width)
            || !(2..=128).contains(&height)
            || !(1..=512).contains(&steps)
            || optics.outer_radius_m > f64::MAX.sqrt() * 0.25
        {
            return Err("transmittance table exceeds its representation bounds");
        }
        let r0 = optics.inner_radius_m;
        let top = optics.outer_radius_m - r0;
        let h = (top * (2. * r0 + top)).sqrt();
        let mut texels = Vec::with_capacity(width as usize * height as usize);
        for y in 0..height {
            let rho = h * f64::from(y) / f64::from(height - 1);
            let radius = (r0 * r0 + rho * rho).sqrt();
            let d_min = optics.outer_radius_m - radius;
            let d_max = rho + h;
            for x in 0..width {
                let distance = d_min + (d_max - d_min) * f64::from(x) / f64::from(width - 1);
                let mu = if distance > 0. {
                    ((h * h - rho * rho - distance * distance) / (2. * radius * distance))
                        .clamp(-1., 1.)
                } else {
                    1.
                };
                let value = unobstructed_transmittance(
                    optics,
                    DVec3::Y * radius,
                    DVec3::new((1. - mu * mu).max(0.).sqrt(), mu, 0.),
                    steps,
                );
                if value
                    .iter()
                    .any(|x| !x.is_finite() || !(0.0..=1.0).contains(x))
                {
                    return Err("non-finite optical table sample");
                }
                texels.push(value.map(|x| x as f32));
            }
        }
        Ok(Self {
            optics: optics.clone(),
            width,
            height,
            steps,
            texels,
        })
    }
    pub fn width(&self) -> u32 {
        self.width
    }
    pub fn height(&self) -> u32 {
        self.height
    }
    pub fn inner_radius_m(&self) -> f64 {
        self.optics.inner_radius_m
    }
    pub fn top_height_m(&self) -> f64 {
        self.optics.outer_radius_m - self.optics.inner_radius_m
    }
    pub fn texels(&self) -> &[[f32; 3]] {
        &self.texels
    }

    /// Emission changes do not invalidate an extinction lookup.
    pub fn matches(&self, optics: &AtmosphereOptics, width: u32, height: u32, steps: u32) -> bool {
        self.width == width
            && self.height == height
            && self.steps == steps
            && self.optics_match(optics)
    }

    pub fn optics_match(&self, optics: &AtmosphereOptics) -> bool {
        self.optics.inner_radius_m == optics.inner_radius_m
            && self.optics.outer_radius_m == optics.outer_radius_m
            && self.optics.rayleigh == optics.rayleigh
            && self.optics.mie == optics.mie
            && self.optics.absorption == optics.absorption
    }

    /// Solar transport at altitude and radial cosine. Below the optical datum,
    /// extrapolate its sea-level boundary; terrain/water own local occlusion.
    pub fn sample(&self, altitude_m: f64, cosine: f64) -> Result<[f64; 3], &'static str> {
        if altitude_m > f64::MAX.sqrt() * 0.25
            || !altitude_m.is_finite()
            || !cosine.is_finite()
            || !(-1.0..=1.0).contains(&cosine)
        {
            return Err("invalid solar lookup coordinates");
        }
        let r0 = self.inner_radius_m();
        let top = self.top_height_m();
        let mut height = altitude_m.max(0.);
        let mut mu = cosine;
        if height > top {
            if mu >= 0. {
                return Ok([1.; 3]);
            }
            let impact = (r0 + height) * (1. - mu * mu).max(0.).sqrt();
            if impact < r0 {
                return Ok([0.; 3]);
            }
            if impact >= r0 + top {
                return Ok([1.; 3]);
            }
            height = top;
            mu = -(1. - (impact / (r0 + top)).powi(2)).max(0.).sqrt();
        }
        let rho = (height * (2. * r0 + height)).sqrt();
        let radius = r0 + height;
        if mu < -rho / radius {
            return Ok([0.; 3]);
        }
        let h = (top * (2. * r0 + top)).sqrt();
        let d_min = top - height;
        let d_max = rho + h;
        let delta = (top - height) * (2. * r0 + top + height);
        let r_mu = radius * mu;
        let root = (r_mu * r_mu + delta).max(0.).sqrt();
        let distance = if r_mu >= 0. && root + r_mu > 0. {
            delta / (root + r_mu)
        } else {
            -r_mu + root
        };
        let x = ((distance - d_min) / (d_max - d_min)).clamp(0., 1.) * f64::from(self.width - 1);
        let y = (rho / h).clamp(0., 1.) * f64::from(self.height - 1);
        let x0 = x.floor() as u32;
        let y0 = y.floor() as u32;
        let get = |x: u32, y: u32| {
            self.texels[(y.min(self.height - 1) * self.width + x.min(self.width - 1)) as usize]
        };
        let a = get(x0, y0);
        let b = get(x0 + 1, y0);
        let c = get(x0, y0 + 1);
        let d = get(x0 + 1, y0 + 1);
        let (fx, fy) = (x - x.floor(), y - y.floor());
        Ok(std::array::from_fn(|i| {
            let lo = f64::from(a[i]) * (1. - fx) + f64::from(b[i]) * fx;
            let hi = f64::from(c[i]) * (1. - fx) + f64::from(d[i]) * fx;
            lo * (1. - fy) + hi * fy
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cached_solar_transport_preserves_the_existing_sky_integral() {
        let optics = crate::nitrogen_oxygen_optics(6_371_000., 101_325., 8000.).unwrap();
        let table = TransmittanceLut::build(&optics, 256, 64, 128).unwrap();
        let mut max_error = 0.0_f64;
        for elevation in [0., 2., 4., 10., 45., 90.] {
            let angle = f64::to_radians(elevation);
            let lights = [crate::CelestialLight {
                direction_to_star: DVec3::new(angle.cos(), angle.sin(), 0.),
                irradiance_w_m2: 1000.,
                color_rgb: [1.; 3],
                angular_radius_rad: 0.005,
                visibility: 1.,
            }];
            for y in [0.02_f64, 0.2, 0.8, 1.] {
                let direction = DVec3::new((1. - y * y).sqrt(), y, 0.);
                let camera = DVec3::Y * (optics.inner_radius_m + 2.);
                let dense = crate::sky_radiance(&optics, camera, direction, &lights, 64, 512);
                let cached =
                    crate::sky_radiance_lut(&optics, camera, direction, &lights, 64, &table)
                        .unwrap();
                assert_eq!(dense.transmittance_rgb, cached.transmittance_rgb);
                for (a, b) in dense.radiance_rgb.into_iter().zip(cached.radiance_rgb) {
                    max_error = max_error.max((a - b).abs() / 1000.);
                }
            }
        }
        eprintln!("cached_sky max_abs_radiance_error_per_unit_stellar_irradiance={max_error}");
        assert!(max_error < 0.0002);
        let mut changed = optics.clone();
        changed.rayleigh.beta_rgb[0] *= 2.;
        assert!(
            crate::sky_radiance_lut(&changed, DVec3::Y * 6_371_002., DVec3::Y, &[], 64, &table)
                .is_none()
        );
    }

    #[test]
    fn solar_lookup_matches_dense_existing_transport_from_ground_to_space() {
        let optics = crate::nitrogen_oxygen_optics(3_200_000., 120_000., 16_300.).unwrap();
        let lut = TransmittanceLut::build(&optics, 256, 64, 128).unwrap();
        let mut maximum = 0.0_f64;
        for height in [0., 0.1, 10., 500., 5000., 20000., 100000., 300000.] {
            for elevation in [-90., -15., -1., 0., 1., 4., 10., 45., 90.] {
                let mu = f64::to_radians(elevation).sin();
                let direction = DVec3::new((1. - mu * mu).max(0.).sqrt(), mu, 0.);
                let expected = crate::transmittance(
                    &optics,
                    DVec3::Y * (optics.inner_radius_m + height),
                    direction,
                    512,
                );
                let actual = lut.sample(height, mu).unwrap();
                for (a, e) in actual.into_iter().zip(expected) {
                    maximum = maximum.max((a - e).abs());
                    assert!(
                        (a - e).abs() < 0.001,
                        "height={height} elevation={elevation} actual={actual:?} expected={expected:?}"
                    );
                }
            }
        }
        eprintln!(
            "solar_lut width=256 height=64 steps=128 dense=512 maximum_absolute_error={maximum}"
        );
        let sunset = lut.sample(2., 4_f64.to_radians().sin()).unwrap();
        assert!(
            sunset[0] > sunset[2],
            "low-angle sunlight must warm through the same atmosphere"
        );
    }
    #[test]
    fn exact_surface_tangent_is_not_an_empty_atmospheric_segment() {
        let optics = crate::nitrogen_oxygen_optics(3_200_000., 120_000., 16_300.).unwrap();
        let ground = crate::transmittance(&optics, DVec3::Y * optics.inner_radius_m, DVec3::X, 512);
        let nearby = crate::transmittance(
            &optics,
            DVec3::Y * (optics.inner_radius_m + 0.01),
            DVec3::X,
            512,
        );
        assert!(ground.iter().all(|v| v.is_finite() && *v < 0.1));
        for (a, b) in ground.into_iter().zip(nearby) {
            assert!((a - b).abs() < 1e-5);
        }
    }
    #[test]
    fn table_bounds_and_emission_independent_cache_are_explicit() {
        let mut optics = crate::nitrogen_oxygen_optics(3_200_000., 120_000., 16_300.).unwrap();
        assert!(TransmittanceLut::build(&optics, 1, 64, 128).is_err());
        let lut = TransmittanceLut::build(&optics, 16, 8, 16).unwrap();
        assert!(lut.sample(f64::NAN, 0.).is_err());
        assert!(lut.sample(f64::MAX, -1.).is_err());
        assert!(lut.sample(0., 2.).is_err());
        optics.emission.airglow_rgb = [2.; 3];
        assert!(lut.matches(&optics, 16, 8, 16));
        optics.rayleigh.beta_rgb[0] *= 2.;
        assert!(!lut.matches(&optics, 16, 8, 16));
    }
}
