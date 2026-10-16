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
    tint: [f64; 3],
}

pub struct CelestialPresentation {
    ephemeris: BakedEphemeris,
    body: BodyId,
    period_s: f64,
    stars: Vec<Star>,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub time: SimTime,
    pub position_inertial_m: DVec3,
    pub frame: BodyFrame,
    pub lighting: SurfaceLighting,
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
            let lux = thessa_atmosphere::irradiance_at_distance(star.luminosity_w, distance) * 93.0;
            stars[index] = (
                lux,
                direction,
                star.tint.map(|channel| (channel * lux) as f32),
            );
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
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
