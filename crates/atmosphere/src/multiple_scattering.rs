//! Isotropic higher-order atmospheric source, following Hillaire's local
//! geometric-series closure. Uses the existing optical/view integrator, with
//! black ground and no terrain/cloud bounce. No source-color fitting or gain.
use crate::{AtmosphereOptics, TransmittanceLut, scattering::isotropic_transfer};
use glam::DVec3;

#[derive(Debug)]
pub struct MultipleScatteringLut {
    optics: AtmosphereOptics,
    width: u32,
    height: u32,
    directions: u32,
    steps: u32,
    source: Vec<[f32; 3]>,
    max_response: f64,
}
impl MultipleScatteringLut {
    pub fn build(
        optics: &AtmosphereOptics,
        solar: &TransmittanceLut,
        width: u32,
        height: u32,
        directions: u32,
        steps: u32,
    ) -> Result<Self, &'static str> {
        if !solar.optics_match(optics) {
            return Err("multiple-scattering solar optics mismatch");
        }
        if !(2..=64).contains(&width)
            || !(2..=64).contains(&height)
            || !(8..=256).contains(&directions)
            || !(4..=256).contains(&steps)
            || u64::from(width) * u64::from(height) * u64::from(directions) * u64::from(steps)
                > 16_777_216
        {
            return Err("multiple-scattering integration exceeds bounded work budget");
        }
        let mut source = Vec::with_capacity((width * height) as usize);
        let mut max_response = 0.0_f64;
        // Deterministic equal-solid-angle midpoint/Fibonacci sphere quadrature.
        let directions: Vec<_> = (0..directions)
            .map(|i| {
                let y = 1. - 2. * (f64::from(i) + 0.5) / f64::from(directions);
                let phase = f64::from(i) * std::f64::consts::PI * (3. - 5.0_f64.sqrt());
                let r = (1. - y * y).sqrt();
                DVec3::new(r * phase.cos(), y, r * phase.sin())
            })
            .collect();
        for row in 0..height {
            let fraction = f64::from(row) / f64::from(height - 1);
            let altitude = solar.top_height_m() * fraction * fraction;
            let p = DVec3::Y * (optics.inner_radius_m + altitude);
            for column in 0..width {
                let mu = 2. * f64::from(column) / f64::from(width - 1) - 1.;
                let sun = DVec3::new((1. - mu * mu).max(0.).sqrt(), mu, 0.);
                let mut first = [0.; 3];
                let mut feedback = [0.; 3];
                for &direction in &directions {
                    let (a, b) = isotropic_transfer(optics, p, direction, sun, solar, steps);
                    for c in 0..3 {
                        first[c] += a[c] / directions.len() as f64;
                        feedback[c] += b[c] / directions.len() as f64;
                    }
                }
                let mut value = [0.; 3];
                for c in 0..3 {
                    if !feedback[c].is_finite()
                        || !(0.0..1.0).contains(&feedback[c])
                        || !first[c].is_finite()
                        || first[c] < 0.
                    {
                        return Err("nonconvergent multiple-scattering response");
                    }
                    max_response = max_response.max(feedback[c]);
                    let radiance = first[c] / (1. - feedback[c]);
                    if !radiance.is_finite() || radiance > f64::from(f32::MAX) {
                        return Err("multiple-scattering source exceeds finite representation");
                    }
                    value[c] = radiance as f32;
                }
                source.push(value);
            }
        }
        Ok(Self {
            optics: optics.clone(),
            width,
            height,
            directions: directions.len() as u32,
            steps,
            source,
            max_response,
        })
    }
    pub fn optics_match(&self, optics: &AtmosphereOptics) -> bool {
        self.optics.inner_radius_m == optics.inner_radius_m
            && self.optics.outer_radius_m == optics.outer_radius_m
            && self.optics.rayleigh == optics.rayleigh
            && self.optics.mie == optics.mie
            && self.optics.absorption == optics.absorption
    }
    pub fn matches(
        &self,
        optics: &AtmosphereOptics,
        width: u32,
        height: u32,
        directions: u32,
        steps: u32,
    ) -> bool {
        self.optics_match(optics)
            && (self.width, self.height, self.directions, self.steps)
                == (width, height, directions, steps)
    }
    pub fn storage_bytes(&self) -> u64 {
        (self.source.len() * std::mem::size_of::<[f32; 3]>()) as u64
    }
    pub fn max_response(&self) -> f64 {
        self.max_response
    }
    pub fn sample(&self, altitude_m: f64, sun_cosine: f64) -> Result<[f64; 3], &'static str> {
        if !altitude_m.is_finite() || !sun_cosine.is_finite() || !(-1.0..=1.0).contains(&sun_cosine)
        {
            return Err("invalid multiple-scattering coordinates");
        }
        if altitude_m > self.optics.outer_radius_m - self.optics.inner_radius_m {
            return Ok([0.; 3]);
        }
        let height =
            (altitude_m.max(0.) / (self.optics.outer_radius_m - self.optics.inner_radius_m)).sqrt();
        let x = (sun_cosine * 0.5 + 0.5) * f64::from(self.width - 1);
        let y = height * f64::from(self.height - 1);
        let a = x.floor() as u32;
        let b = y.floor() as u32;
        let fx = x - x.floor();
        let fy = y - y.floor();
        let get = |x: u32, y: u32| {
            self.source[(y.min(self.height - 1) * self.width + x.min(self.width - 1)) as usize]
        };
        let [p, q, r, s] = [get(a, b), get(a + 1, b), get(a, b + 1), get(a + 1, b + 1)];
        Ok(std::array::from_fn(|i| {
            (f64::from(p[i]) * (1. - fx) + f64::from(q[i]) * fx) * (1. - fy)
                + (f64::from(r[i]) * (1. - fx) + f64::from(s[i]) * fx) * fy
        }))
    }
    pub(crate) fn sample_point(&self, p: DVec3, sun: DVec3) -> [f64; 3] {
        let radius = p.length();
        self.sample(
            radius - self.optics.inner_radius_m,
            (p / radius).dot(sun).clamp(-1., 1.),
        )
        .expect("finite shared atmospheric integration point")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CelestialLight, sky_radiance_lut, sky_radiance_lut_with_multiple};
    fn optics() -> AtmosphereOptics {
        let mut optics = crate::nitrogen_oxygen_optics(3_200_000., 120_000., 16_300.).unwrap();
        optics.emission.airglow_rgb = [0.; 3];
        optics
    }
    fn light(mu: f64, rgb: [f64; 3]) -> CelestialLight {
        CelestialLight {
            direction_to_star: DVec3::new((1. - mu * mu).sqrt(), mu, 0.),
            irradiance_w_m2: 1000.,
            color_rgb: rgb,
            angular_radius_rad: 0.005,
            visibility: 1.,
        }
    }
    #[test]
    fn finite_cell_source_preserves_vacuum_composition_and_half_absorption() {
        use crate::scattering::cell_source_integral;
        assert_eq!(cell_source_integral(0., 3.), 3.);
        let k = std::f64::consts::LN_2;
        assert!((cell_source_integral(k, 1.) - 0.5 / k).abs() < 1e-14);
        assert!((cell_source_integral(1e-15, 1.) - 1.).abs() < 1e-14);
        for k in [0., 1e-9, 0.001, 10.] {
            let a = cell_source_integral(k, 20.) + (-k * 20.).exp() * cell_source_integral(k, 80.);
            assert!((a - cell_source_integral(k, 100.)).abs() < 1e-11);
        }
    }
    #[test]
    fn no_scatter_is_zero_and_cache_rejects_wrong_domains_and_unbounded_work() {
        let mut optics = optics();
        optics.rayleigh.beta_rgb = [0.; 3];
        optics.mie.beta_rgb = [0.; 3];
        let solar = TransmittanceLut::build(&optics, 32, 16, 32).unwrap();
        let ms = MultipleScatteringLut::build(&optics, &solar, 4, 4, 8, 8).unwrap();
        assert_eq!(ms.max_response(), 0.);
        assert!(ms.source.iter().all(|x| *x == [0.; 3]));
        assert!(ms.sample(0., f64::NAN).is_err());
        assert!(ms.sample(f64::INFINITY, 0.).is_err());
        assert_eq!(ms.sample(1e7, 0.).unwrap(), [0.; 3]);
        assert!(MultipleScatteringLut::build(&optics, &solar, 64, 64, 256, 256).is_err());
        optics.emission.airglow_rgb = [1., 2., 3.];
        assert!(ms.optics_match(&optics));
        optics.absorption[0].sigma_rgb[0] *= 2.;
        assert!(!ms.optics_match(&optics));
        assert!(MultipleScatteringLut::build(&optics, &solar, 4, 4, 8, 8).is_err());
    }
    #[test]
    fn shared_multiple_transport_is_additive_eclipse_aware_and_keeps_extinction() {
        let optics = optics();
        let solar = TransmittanceLut::build(&optics, 128, 32, 128).unwrap();
        let ms = MultipleScatteringLut::build(&optics, &solar, 16, 8, 32, 32).unwrap();
        assert!(ms.max_response() > 0. && ms.max_response() < 1.);
        let camera = DVec3::Y * (optics.inner_radius_m + 2.);
        let view = DVec3::new(0., 0.7, 0.7).normalize();
        let stars = [
            light(0.07, [1., 0.5, 0.2]),
            light(0.7, [0.2, 0.5, 1.]),
            light(-0.02, [0.1, 0.2, 0.1]),
        ];
        let evaluate = |stars: &[CelestialLight], multiple| {
            sky_radiance_lut_with_multiple(&optics, camera, view, stars, 64, &solar, multiple)
                .unwrap()
        };
        let single = sky_radiance_lut(&optics, camera, view, &stars, 64, &solar).unwrap();
        let off = evaluate(&stars, None);
        assert_eq!(single.radiance_rgb, off.radiance_rgb);
        let all = evaluate(&stars, Some(&ms));
        assert_eq!(single.transmittance_rgb, all.transmittance_rgb);
        let mut sum = [0.; 3];
        for star in &stars {
            let value = evaluate(std::slice::from_ref(star), Some(&ms));
            for (sum, value) in sum.iter_mut().zip(value.radiance_rgb) {
                *sum += value;
            }
        }
        for (c, sum) in sum.iter().enumerate() {
            assert!(all.radiance_rgb[c] > single.radiance_rgb[c]);
            assert!((all.radiance_rgb[c] - sum).abs() < 1e-10);
        }
        let mut eclipse = stars[0].clone();
        eclipse.visibility = 0.;
        assert_eq!(
            evaluate(&[eclipse.clone()], Some(&ms)).radiance_rgb,
            [0.; 3]
        );
        eclipse.visibility = 0.25;
        let a = evaluate(&[eclipse], Some(&ms));
        let b = evaluate(&stars[..1], Some(&ms));
        for c in 0..3 {
            assert!((a.radiance_rgb[c] - b.radiance_rgb[c] * 0.25).abs() < 1e-11);
        }
        // Rotation cannot change a radially symmetric atmosphere's result.
        let rotate = glam::DQuat::from_axis_angle(DVec3::new(1., 2., 3.).normalize(), 1.3);
        let turned: Vec<_> = stars
            .iter()
            .map(|s| {
                let mut s = s.clone();
                s.direction_to_star = rotate * s.direction_to_star;
                s
            })
            .collect();
        let rotated = sky_radiance_lut_with_multiple(
            &optics,
            rotate * camera,
            rotate * view,
            &turned,
            64,
            &solar,
            Some(&ms),
        )
        .unwrap();
        for c in 0..3 {
            assert!((rotated.radiance_rgb[c] - all.radiance_rgb[c]).abs() < 1e-7);
        }
    }
    #[test]
    fn lookup_converges_against_dense_isotropic_quadrature() {
        let optics = optics();
        let solar = TransmittanceLut::build(&optics, 256, 64, 128).unwrap();
        let started = std::time::Instant::now();
        let table = MultipleScatteringLut::build(&optics, &solar, 32, 16, 64, 64).unwrap();
        let build_ms = started.elapsed().as_secs_f64() * 1000.;
        let high = MultipleScatteringLut::build(&optics, &solar, 64, 32, 64, 32).unwrap();
        let mut max_error = 0.0_f64;
        let mut high_error = 0.0_f64;
        for altitude in [2., 10_000.] {
            for mu in [0.0, 0.07, 0.7] {
                let p = DVec3::Y * (optics.inner_radius_m + altitude);
                let sun = DVec3::new((1.0_f64 - mu * mu).sqrt(), mu, 0.);
                // Independent tensor-product solid-angle quadrature, not the table's
                // Fibonacci directions. Dense reference of the declared closure.
                let mut source = [0.; 3];
                let mut feedback = [0.; 3];
                for row in 0..16 {
                    for column in 0..32 {
                        let y = 1. - 2. * (f64::from(row) + 0.5) / 16.;
                        let phi = std::f64::consts::TAU * (f64::from(column) + 0.5) / 32.;
                        let r = (1. - y * y).sqrt();
                        let (a, b) = isotropic_transfer(
                            &optics,
                            p,
                            DVec3::new(r * phi.cos(), y, r * phi.sin()),
                            sun,
                            &solar,
                            128,
                        );
                        for c in 0..3 {
                            source[c] += a[c] / 512.;
                            feedback[c] += b[c] / 512.;
                        }
                    }
                }
                let sampled = table.sample(altitude, mu).unwrap();
                let sampled_high = high.sample(altitude, mu).unwrap();
                for c in 0..3 {
                    let dense = source[c] / (1. - feedback[c]);
                    max_error = max_error.max((sampled[c] - dense).abs());
                    high_error = high_error.max((sampled_high[c] - dense).abs());
                }
            }
        }
        eprintln!(
            "multiple_lookup source_knots=32x16 directions=64 steps=64 bytes={} build_ms={build_ms} max_abs_error_per_unit_irradiance={max_error} max_feedback={} high_64x32_64directions_32steps_error={high_error}",
            table.storage_bytes(),
            table.max_response()
        );
        assert!(
            high_error < 0.0015,
            "High closure lookup error {high_error}"
        );
        assert!(
            max_error < 0.0015,
            "closure quadrature/lookup error {max_error}"
        );
    }
}
