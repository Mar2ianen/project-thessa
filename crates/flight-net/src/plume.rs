//! Accepted rocket interval readout. No propulsion solve or renderer dependency.
use serde::{Deserialize, Serialize};
use thessa_flight_authority::FlightAuthority;
use thessa_sim_core::{BakedEphemeris, GasKind, RocketNozzleObservation, SimTime, VehicleId};

/// Transport bound; unsupported publications fail rather than truncate engines.
pub const MAX_ROCKET_NOZZLES: usize = 1024;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RocketPlumeSample {
    pub vehicle_id: VehicleId,
    pub interval_end: SimTime,
    pub duration_s: f64,
    /// Pressure used by the accepted operating-point solve.
    pub pressure_pa: f64,
    /// Remaining ambient data sampled at the accepted COM endpoint.
    pub density_kg_m3: f64,
    pub temperature_k: f64,
    pub oxygen_mole_fraction: f64,
    /// Air minus COM velocity and air rotation in inertial axes.
    pub air_velocity_inertial_mps: [f64; 3],
    pub air_rotation_inertial_rps: [f64; 3],
    pub nozzles: Vec<RocketNozzleObservation>,
}

impl RocketPlumeSample {
    pub fn from_authority(
        authority: &FlightAuthority,
        ephemeris: &BakedEphemeris,
        vehicle_id: VehicleId,
    ) -> Option<Self> {
        let (end, observation) = authority.last_propulsion_observation.as_ref()?;
        // Rails/other advances must not relabel an old burn as a fresh interval.
        if end.0 != authority.flight_time_s {
            return None;
        }
        let body = ephemeris.body_state(authority.reference_body, *end).ok()?;
        let air = thessa_flight_authority::local_air_kinematics(
            authority.atmosphere,
            authority.state,
            body,
            authority.planet_radius_m,
        )
        .ok()?;
        let ambient = authority.atmosphere.sample(air.altitude_m.max(0.0)).ok()?;
        let sample = Self {
            vehicle_id,
            interval_end: *end,
            duration_s: observation.duration_s,
            pressure_pa: observation.ambient_pressure_pa,
            density_kg_m3: ambient.density_kg_m3,
            temperature_k: ambient.temperature_k,
            oxygen_mole_fraction: authority
                .atmosphere
                .composition
                .mole_fraction(GasKind::Oxygen),
            air_velocity_inertial_mps: (-(authority.state.orientation_body_to_inertial
                * air.air_velocity_body_mps))
                .to_array(),
            air_rotation_inertial_rps: authority.atmosphere.body_rotation_rad_s.to_array(),
            nozzles: observation.nozzles.clone()?,
        };
        sample.validate(vehicle_id).ok()?;
        Some(sample)
    }

    pub fn validate(&self, owner: VehicleId) -> Result<(), String> {
        let nonnegative = |v: f64| v.is_finite() && v >= 0.0;
        if self.vehicle_id != owner
            || !nonnegative(self.interval_end.0)
            || !self.duration_s.is_finite()
            || self.duration_s <= 0.0
            || !nonnegative(self.pressure_pa)
            || !nonnegative(self.density_kg_m3)
            || !nonnegative(self.temperature_k)
            || !self.oxygen_mole_fraction.is_finite()
            || !(0.0..=1.0).contains(&self.oxygen_mole_fraction)
            || self.nozzles.len() > MAX_ROCKET_NOZZLES
            || self
                .air_velocity_inertial_mps
                .iter()
                .chain(&self.air_rotation_inertial_rps)
                .any(|v| !v.is_finite())
        {
            return Err("invalid accepted rocket interval/owner/ambient".into());
        }
        let mut identities = std::collections::BTreeSet::new();
        for nozzle in &self.nozzles {
            let key = match nozzle.id {
                thessa_sim_core::RocketNozzleId::Engine(i) => (0, i, 0),
                thessa_sim_core::RocketNozzleId::Chamber { system, chamber } => {
                    (1, system, chamber)
                }
            };
            if !identities.insert(key)
                || !nozzle.gamma.is_finite()
                || !(1.0..=2.0).contains(&nozzle.gamma)
                || nozzle.gamma == 1.0
                || !nozzle.active_fraction.is_finite()
                || !(0.0..=1.0).contains(&nozzle.active_fraction)
            {
                return Err("invalid/duplicate accepted rocket nozzle".into());
            }
            if let Some(outlet) = nozzle.outlet {
                let axis = glam::DVec3::from_array(outlet.exhaust_axis_body);
                if !glam::DVec3::from_array(outlet.position_body_m).is_finite()
                    || !axis.is_finite()
                    || (axis.length_squared() - 1.0).abs() > 2e-9
                    || !outlet.radius_m.is_finite()
                    || outlet.radius_m <= 0.0
                {
                    return Err("invalid accepted rocket outlet".into());
                }
            }
            if let Some(state) = nozzle.state
                && (![
                    state.mass_flow_kg_s,
                    state.exhaust_velocity_mps,
                    state.exit_pressure_pa,
                    state.exit_temp_k,
                    state.exit_mach,
                ]
                .into_iter()
                .all(nonnegative)
                    || !state.exit_radius_m.is_finite()
                    || state.exit_radius_m <= 0.0
                    || nozzle
                        .outlet
                        .is_some_and(|o| o.radius_m != state.exit_radius_m))
            {
                return Err("invalid accepted rocket operating state".into());
            }
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use thessa_sim_core::{EnginePlumeState, NozzleOutlet, Propellant, RocketNozzleId};

    pub(crate) fn sample() -> RocketPlumeSample {
        RocketPlumeSample {
            vehicle_id: VehicleId::PRIMARY,
            interval_end: SimTime(60.),
            duration_s: 1. / 120.,
            pressure_pa: 101325.,
            density_kg_m3: 1.2,
            temperature_k: 290.,
            oxygen_mole_fraction: 0.21,
            air_velocity_inertial_mps: [1., 2., 3.],
            air_rotation_inertial_rps: [0., 0., 0.0001],
            nozzles: vec![RocketNozzleObservation {
                id: RocketNozzleId::Engine(0),
                gamma: 1.22,
                active_fraction: 0.25,
                outlet: Some(NozzleOutlet {
                    position_body_m: [-2., 1., 0.],
                    exhaust_axis_body: [-1., 0., 0.],
                    radius_m: 0.55,
                }),
                state: Some(EnginePlumeState {
                    exit_radius_m: 0.55,
                    mass_flow_kg_s: 120.,
                    exhaust_velocity_mps: 3550.,
                    exit_pressure_pa: 120000.,
                    exit_temp_k: 1850.,
                    exit_mach: 3.5,
                    propellant: Propellant::LoxMethane,
                }),
            }],
        }
    }

    #[test]
    fn accepted_interval_rejects_wrong_owner_duplicate_ids_and_invalid_geometry() {
        let s = sample();
        s.validate(VehicleId::PRIMARY).unwrap();
        assert!(s.validate(VehicleId::new(1)).is_err());
        let mut duplicate = s.clone();
        duplicate.nozzles.push(s.nozzles[0]);
        assert!(duplicate.validate(VehicleId::PRIMARY).is_err());
        let mut invalid = s.clone();
        invalid.nozzles[0]
            .outlet
            .as_mut()
            .unwrap()
            .exhaust_axis_body = [0.; 3];
        assert!(invalid.validate(VehicleId::PRIMARY).is_err());
        let mut invalid = s.clone();
        invalid.nozzles[0].state.as_mut().unwrap().mass_flow_kg_s = f64::NAN;
        assert!(invalid.validate(VehicleId::PRIMARY).is_err());
        let mut missing = s;
        missing.nozzles[0].outlet = None;
        missing.validate(VehicleId::PRIMARY).unwrap();
    }
}
