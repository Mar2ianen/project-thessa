//! Presentation samples from the existing baked ephemeris and flight spin
//! convention. No integration, system clock, or new authoritative state.
use glam::DVec3;
use thessa_flight_authority::presentation::{
    inertial_to_render_axes, terrain_rotation, terrain_rotation_period_s,
};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::BodyFrame;
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, SystemConfig};

struct Star {
    id: BodyId,
    luminosity_w: f64,
    radius_m: f64,
    tint: [f64; 3],
}

pub struct CelestialPresentation {
    ephemeris: BakedEphemeris,
    body: BodyId,
    period_s: f64,
    stars: Vec<Star>,
}

#[derive(Clone, Debug)]
pub struct Sample {
    pub time: SimTime,
    pub position_inertial_m: DVec3,
    pub frame: BodyFrame,
    pub lighting: SurfaceLighting,
    /// Stellar sources in body axes with physical irradiance, for the shared
    /// scattering model. Separate from `lighting`, which is already scaled to
    /// the presentation's photometric units for shading.
    pub environment_lights: Vec<thessa_atmosphere::CelestialLight>,
}

impl CelestialPresentation {
    pub fn ephemeris(&self) -> &BakedEphemeris {
        &self.ephemeris
    }
    pub fn initial_flight(&self) -> Result<thessa_flight_authority::FlightAuthority, String> {
        thessa_flight_authority::FlightAuthority::new(&self.ephemeris, self.body)
    }
    pub fn new(
        system: &SystemConfig,
        ephemeris: BakedEphemeris,
        body: BodyId,
    ) -> Result<Self, String> {
        let definition = ephemeris
            .bodies
            .iter()
            .find(|b| b.id == body)
            .ok_or("missing terrain body")?;
        let period_s = terrain_rotation_period_s(definition);
        if !period_s.is_finite() || period_s <= 0.0 {
            return Err("invalid terrain rotation period".into());
        }
        if system.star.len() > 3 {
            return Err(
                "terrain shader currently supports at most three direct stellar sources".into(),
            );
        }
        let stars = system
            .star
            .iter()
            .map(|star| {
                Ok(Star {
                    id: ephemeris.body_id(&star.id).ok_or("missing star")?,
                    luminosity_w: star.luminosity_solar * 3.828e26,
                    radius_m: star.radius_solar * 6.957e8,
                    tint: thessa_atmosphere::blackbody_rgb(star.temperature_k),
                })
            })
            .collect::<Result<_, String>>()?;
        Ok(Self {
            ephemeris,
            body,
            period_s,
            stars,
        })
    }

    /// Locate a descending solar elevation using the baked ephemeris and the
    /// existing body spin. This selects a reproducible presentation epoch only.
    pub fn descending_sun_epoch(&self, up: DVec3, elevation_rad: f64) -> Result<SimTime, String> {
        if !up.is_finite()
            || (up.length_squared() - 1.0).abs() > 1e-8
            || !elevation_rad.is_finite()
            || !(0.0..std::f64::consts::FRAC_PI_2).contains(&elevation_rad)
        {
            return Err("invalid sunset search coordinates".into());
        }
        let target = elevation_rad.sin();
        let elevation = |t| -> Result<f64, String> {
            let sample = self.sample(SimTime(t))?;
            if sample.lighting.colors_lux[0][..3].iter().all(|v| *v <= 0.0) {
                return Err("sunset requires a stellar source".into());
            }
            let d = sample.lighting.directions[0];
            Ok(up.dot(DVec3::new(
                f64::from(d[0]),
                f64::from(d[1]),
                f64::from(d[2]),
            )) - target)
        };
        let mut previous = elevation(0.0)?;
        for i in 1..=384 {
            let end = self.period_s * 2.0 * f64::from(i) / 384.0;
            let current = elevation(end)?;
            if previous >= 0.0 && current <= 0.0 {
                let mut lo = self.period_s * 2.0 * f64::from(i - 1) / 384.0;
                let mut hi = end;
                for _ in 0..48 {
                    let mid = (lo + hi) * 0.5;
                    if elevation(mid)? > 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                return Ok(SimTime((lo + hi) * 0.5));
            }
            previous = current;
        }
        Err("no descending solar crossing in two body rotations".into())
    }

    pub fn sample(&self, time: SimTime) -> Result<Sample, String> {
        if !time.0.is_finite() {
            return Err("non-finite presentation simulation time".into());
        }
        let planet = self
            .ephemeris
            .body_state(self.body, time)
            .map_err(|e| e.to_string())?;
        let position = inertial_to_render_axes(planet.position_inertial);
        // Keep the origin attached to the reference body. Never form an
        // absolute camera position by adding a metre-scale offset to its orbit.
        let frame = BodyFrame::new(position, terrain_rotation(time, self.period_s), position)
            .ok_or("invalid celestial presentation pose")?;
        let mut stars = [(0.0_f64, DVec3::ZERO, [0.0_f32; 3]); 3];
        let mut environment_lights = Vec::with_capacity(self.stars.len());
        for (index, star) in self.stars.iter().enumerate() {
            let state = self
                .ephemeris
                .body_state(star.id, time)
                .map_err(|e| e.to_string())?;
            let delta = state.position_inertial - planet.position_inertial;
            let distance = delta.length();
            if !delta.is_finite() || !distance.is_finite() || distance <= 0.0 {
                return Err("invalid stellar displacement".into());
            }
            let direction =
                frame.render_to_body_direction(inertial_to_render_axes(delta.normalize()));
            let irradiance_w_m2 =
                thessa_atmosphere::irradiance_at_distance(star.luminosity_w, distance);
            let lux = irradiance_w_m2 * crate::environment::LUX_PER_W_M2;
            stars[index] = (
                lux,
                direction,
                star.tint.map(|channel| (channel * lux) as f32),
            );
            environment_lights.push(thessa_atmosphere::CelestialLight {
                direction_to_star: direction,
                irradiance_w_m2,
                color_rgb: star.tint,
                angular_radius_rad: thessa_atmosphere::angular_radius(star.radius_m, distance),
                // Eclipses are not part of this presentation yet: the native
                // path must not claim an eclipse it does not model.
                visibility: 1.0,
            });
        }
        stars[..self.stars.len()].sort_by(|a, b| b.0.total_cmp(&a.0));
        let mut lighting = SurfaceLighting::default();
        for (i, (_, direction, color)) in stars.into_iter().take(self.stars.len()).enumerate() {
            lighting.directions[i] = [
                direction.x as f32,
                direction.y as f32,
                direction.z as f32,
                0.0,
            ];
            lighting.colors_lux[i] = [color[0], color[1], color[2], 0.0];
        }
        if lighting.records().is_none() {
            return Err("invalid celestial surface lighting".into());
        }
        Ok(Sample {
            time,
            position_inertial_m: planet.position_inertial,
            frame,
            lighting,
            environment_lights,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sunset_search_samples_existing_ephemeris_and_descending_spin() {
        let system: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let body = ephemeris.body_id("thessa").unwrap();
        let p = CelestialPresentation::new(&system, ephemeris, body).unwrap();
        let lat = thessa_reference_scenes::B03_SURFACE_SHOWCASE_LAT_DEG.to_radians();
        let lon = thessa_reference_scenes::B03_SURFACE_SHOWCASE_LON_DEG.to_radians();
        let up = DVec3::new(lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin());
        let target = 4.0_f64.to_radians();
        let time = p.descending_sun_epoch(up, target).unwrap();
        let cosine = |t| {
            let d = p.sample(t).unwrap().lighting.directions[0];
            up.dot(DVec3::new(
                f64::from(d[0]),
                f64::from(d[1]),
                f64::from(d[2]),
            ))
        };
        assert!((cosine(time) - target.sin()).abs() < 1e-6);
        assert!(cosine(SimTime(time.0 - 1.0)) > cosine(SimTime(time.0 + 1.0)));
        assert_eq!(time, p.descending_sun_epoch(up, target).unwrap());
        for degrees in [4., 8., 12.] {
            let epoch = p
                .descending_sun_epoch(up, f64::to_radians(degrees))
                .unwrap();
            eprintln!(
                "B03 solar_elevation_deg={degrees} presentation_sim_time_s={}",
                epoch.0
            );
            assert!((cosine(epoch) - f64::to_radians(degrees).sin()).abs() < 1e-6);
        }
        assert!(p.descending_sun_epoch(DVec3::ZERO, target).is_err());
        eprintln!("B03 descending 4-degree sun epoch_s={}", time.0);
    }

    #[test]
    fn canonical_translation_spin_and_lighting_follow_the_same_simulation_sample() {
        let system: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let body = ephemeris.body_id("thessa").unwrap();
        let presentation = CelestialPresentation::new(&system, ephemeris, body).unwrap();
        let zero = presentation.sample(SimTime::EPOCH).unwrap();
        let later = presentation.sample(SimTime(72_000.0)).unwrap();
        assert!((later.position_inertial_m - zero.position_inertial_m).length() > 1e8);
        assert!((later.frame.rotation() * DVec3::Z - DVec3::X).length() < 1e-12);
        assert_ne!(zero.lighting.directions, later.lighting.directions);
        let camera_body = DVec3::new(0.03125, 3_230_000.0, -0.015625);
        for sample in [zero, later] {
            let local = sample.frame.body_to_render_point(camera_body);
            assert!((sample.frame.render_to_body_point(local) - camera_body).length() < 3e-9);
        }
        for time in [f64::NAN, f64::INFINITY] {
            assert!(presentation.sample(SimTime(time)).is_err());
        }
    }
}
