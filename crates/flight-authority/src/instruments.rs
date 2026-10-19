//! Read-only pilot telemetry shared by native and retained hosts. No stepping,
//! control writes, sensor fiction or renderer/transport handles.
use glam::DVec3;
use thessa_sim_core::{BakedEphemeris, BodyId, ParachuteLoad, ParachutePhase, SimTime};

use crate::{FlightAuthority, local_air_kinematics, navball::InstrumentFrame};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum SpeedFrame {
    #[default]
    Surface,
    Air,
    Orbital,
    Target,
}
impl SpeedFrame {
    pub const ALL: [Self; 4] = [Self::Surface, Self::Air, Self::Orbital, Self::Target];
    pub fn next(self) -> Self {
        Self::ALL[(Self::ALL.iter().position(|frame| *frame == self).unwrap() + 1) % 4]
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Surface => "SURFACE",
            Self::Air => "AIR",
            Self::Orbital => "ORBITAL",
            Self::Target => "TARGET",
        }
    }
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum AltitudeFrame {
    #[default]
    Datum,
    Agl,
}
impl AltitudeFrame {
    pub fn toggle(self) -> Self {
        match self {
            Self::Datum => Self::Agl,
            Self::Agl => Self::Datum,
        }
    }
}

/// Replica forces/environment reconstructed for display are NOT observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    LocalAuthority,
    Replica,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirData {
    pub velocity_body_mps: DVec3,
    pub angle_of_attack_deg: f64,
    pub sideslip_deg: f64,
    pub mach: Option<f64>,
    pub dynamic_pressure_pa: Option<f64>,
    pub pressure_pa: Option<f64>,
    pub density_kg_m3: Option<f64>,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadData {
    pub thrust_n: f64,
    /// Non-gravitational force / mass / standard gravity, not gravity itself.
    pub g_load: f64,
    /// Retains the pilot's central-body gravity reference, not summed-field g.
    pub twr: f64,
}
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ParachuteSummary {
    pub installed: usize,
    pub armed: usize,
    pub open: usize,
    pub failed: usize,
    pub drag_n: f64,
    pub dynamic_pressure_pa: f64,
}
impl ParachuteSummary {
    /// Reuses the authority's adoption validator; malformed/mismatched wire
    /// vectors must not be replaced by the replica's previous/local canopy state.
    pub fn checked(flight: &FlightAuthority, loads: &[ParachuteLoad]) -> Option<Self> {
        if !flight.parachute_telemetry_valid(loads) {
            return None;
        }
        let mut result = Self {
            installed: loads.len(),
            ..Self::default()
        };
        for load in loads {
            result.armed += usize::from(load.state.phase == ParachutePhase::Armed);
            result.open += usize::from(matches!(
                load.state.phase,
                ParachutePhase::Reefed | ParachutePhase::Deployed
            ));
            result.failed += usize::from(load.state.phase == ParachutePhase::Failed);
            result.drag_n += load.force_body_n.length();
            result.dynamic_pressure_pa = result.dynamic_pressure_pa.max(load.dynamic_pressure_pa);
        }
        result.drag_n.is_finite().then_some(result)
    }
}

/// One newest-authority epoch, SI/f64. AGL is absent until the host supplies a
/// matching body-owned terrain sample. Changing this display hint never changes
/// the reference body, gravity, control law or authoritative state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightInstruments {
    pub time: SimTime,
    pub display_body: BodyId,
    pub at_reference_body: bool,
    pub surface_speed_mps: f64,
    pub orbital_speed_mps: f64,
    pub vertical_speed_mps: f64,
    pub altitude_datum_m: f64,
    pub altitude_agl_m: Option<f64>,
    pub local_up_body: DVec3,
    pub pole_body: DVec3,
    pub surface_velocity_body: DVec3,
    pub orbital_velocity_body: DVec3,
    pub air: Option<AirData>,
    pub loads: Option<LoadData>,
    pub apoapsis_altitude_m: Option<f64>,
    pub periapsis_altitude_m: Option<f64>,
    pub reaction_wheel_torque_body_nm: DVec3,
    pub parachutes: Option<ParachuteSummary>,
}
impl FlightInstruments {
    pub fn speed(&self, frame: SpeedFrame) -> Option<f64> {
        self.velocity_body(frame).map(|v| v.length())
    }
    pub fn velocity_body(&self, frame: SpeedFrame) -> Option<DVec3> {
        match frame {
            SpeedFrame::Surface => Some(self.surface_velocity_body),
            SpeedFrame::Orbital => Some(self.orbital_velocity_body),
            SpeedFrame::Air => self.air.map(|air| air.velocity_body_mps),
            SpeedFrame::Target => None,
        }
    }
    pub fn altitude(&self, frame: AltitudeFrame) -> Option<f64> {
        match frame {
            AltitudeFrame::Datum => Some(self.altitude_datum_m),
            AltitudeFrame::Agl => self.altitude_agl_m,
        }
    }
    pub fn attitude(&self) -> InstrumentFrame {
        InstrumentFrame::new(
            self.local_up_body,
            self.pole_body,
            self.surface_velocity_body,
        )
    }
}

pub fn sample(
    flight: &FlightAuthority,
    ephemeris: &BakedEphemeris,
    source: Source,
) -> Option<FlightInstruments> {
    if !flight.flight_time_s.is_finite()
        || !flight.state.position_inertial_m.is_finite()
        || !flight.state.velocity_inertial_mps.is_finite()
        || !flight.state.orientation_body_to_inertial.is_normalized()
    {
        return None;
    }
    let time = SimTime(flight.flight_time_s);
    let display_body = crate::navigation::craft_display_body(ephemeris, flight, time);
    let body = ephemeris.body(display_body).ok()?;
    let body_state = ephemeris.body_state(display_body, time).ok()?;
    let k =
        local_air_kinematics(flight.atmosphere, flight.state, body_state, body.radius_m).ok()?;
    let at_home = display_body == flight.reference_body;
    // The only installed atmosphere/rotation provider belongs to the launch
    // body. Away from home preserve the legacy body-relative fallback, but do
    // not apply the launch body's rotation, air or terrain to another world.
    let surface_velocity = if at_home {
        k.surface_velocity_inertial_mps
    } else {
        k.relative_velocity_inertial_mps
    };
    let inverse = flight.state.orientation_body_to_inertial.inverse();
    let local = source == Source::LocalAuthority;
    let forces = local.then_some(flight.last_forces.as_ref()).flatten();
    let air = (local && at_home).then(|| {
        let environment = forces.map(|f| f.environment);
        AirData {
            velocity_body_mps: k.air_velocity_body_mps,
            angle_of_attack_deg: crate::conventional_angle_of_attack_deg(k.air_velocity_body_mps),
            sideslip_deg: k
                .air_velocity_body_mps
                .y
                .atan2(k.air_velocity_body_mps.x.abs().max(1e-6))
                .to_degrees(),
            mach: forces.map(|f| f.aero.mach),
            dynamic_pressure_pa: forces.map(|f| f.aero.dynamic_pressure_pa),
            pressure_pa: environment.map(|e| {
                e.density_kg_m3 * e.speed_of_sound_mps.powi(2)
                    / flight.atmosphere.heat_capacity_ratio
            }),
            density_kg_m3: environment.map(|e| e.density_kg_m3),
        }
    });
    let elements = crate::navigation::craft_elements(ephemeris, flight, time);
    let mass = flight.vehicle.mass_properties.mass_kg;
    let loads = forces.map(|f| LoadData {
        thrust_n: flight.thrust_n(),
        g_load: f.total_force_inertial_n.length() / mass / 9.80665,
        twr: flight.thrust_n() / (mass * body.mu / k.relative_position_inertial_m.length_squared()),
    });
    Some(FlightInstruments {
        time,
        display_body,
        at_reference_body: at_home,
        surface_speed_mps: surface_velocity.length(),
        orbital_speed_mps: k.relative_velocity_inertial_mps.length(),
        vertical_speed_mps: surface_velocity.dot(k.radial_up),
        altitude_datum_m: k.altitude_m.max(0.0),
        altitude_agl_m: None,
        local_up_body: inverse * k.radial_up,
        pole_body: inverse
            * if at_home {
                flight
                    .atmosphere
                    .body_rotation_rad_s
                    .try_normalize()
                    .unwrap_or(DVec3::Z)
            } else {
                DVec3::Z
            },
        surface_velocity_body: inverse * surface_velocity,
        orbital_velocity_body: inverse * k.relative_velocity_inertial_mps,
        air,
        loads,
        apoapsis_altitude_m: elements
            .and_then(|e| e.apoapsis_m())
            .map(|r| r - body.radius_m),
        periapsis_altitude_m: elements.map(|e| e.periapsis_m() - body.radius_m),
        reaction_wheel_torque_body_nm: flight.reaction_wheel_telemetry(),
        parachutes: if local {
            ParachuteSummary::checked(flight, flight.parachute_telemetry())
        } else {
            None
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;

    fn fixture() -> (BakedEphemeris, FlightAuthority) {
        let config: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let e = config.bake().unwrap();
        let flight = FlightAuthority::new(&e, e.body_id("thessa").unwrap()).unwrap();
        (e, flight)
    }

    #[test]
    fn instruments_share_own_epoch_body_and_orbit_math_without_changing_physics() {
        let (e, mut flight) = fixture();
        let reference = flight.reference_body;
        for time in [0.0, 12345.0] {
            flight.flight_time_s = time;
            for name in ["thessa", "nereid"] {
                let id = e.body_id(name).unwrap();
                let body = e.body(id).unwrap();
                let origin = e.body_state(id, SimTime(time)).unwrap();
                let radius = body.radius_m + 500_000.0;
                let relative = DVec3::X * radius;
                let velocity = DVec3::Y * (body.mu / radius).sqrt();
                flight.state.position_inertial_m = origin.position_inertial + relative;
                flight.state.velocity_inertial_mps = origin.velocity_inertial + velocity;
                flight.state.orientation_body_to_inertial = DQuat::from_rotation_x(0.4);
                let before = flight.state;
                let data = sample(&flight, &e, Source::LocalAuthority).unwrap();
                assert_eq!(data.display_body, id);
                assert_eq!(data.time, SimTime(time));
                assert!((data.altitude_datum_m - 500_000.0).abs() < 0.001);
                assert!((data.orbital_speed_mps - velocity.length()).abs() < 0.001);
                assert!((data.apoapsis_altitude_m.unwrap() - 500_000.0).abs() < 0.02);
                assert!((data.periapsis_altitude_m.unwrap() - 500_000.0).abs() < 0.02);
                let expected = if id == reference {
                    velocity - flight.atmosphere.body_rotation_rad_s.cross(relative)
                } else {
                    velocity
                };
                assert!((data.surface_speed_mps - expected.length()).abs() < 0.001);
                assert!(
                    (data.surface_velocity_body
                        - flight.state.orientation_body_to_inertial.inverse() * expected)
                        .length()
                        < 0.001
                );
                assert!(data.altitude_agl_m.is_none());
                assert_eq!(data.air.is_some(), id == reference);
                assert_eq!(data.speed(SpeedFrame::Target), None);
                assert_eq!(data.velocity_body(SpeedFrame::Target), None);
                assert_eq!(flight.state, before);
                assert_eq!(flight.reference_body, reference);
            }
        }
        flight.state.position_inertial_m = DVec3::NAN;
        assert!(sample(&flight, &e, Source::LocalAuthority).is_none());
    }

    #[test]
    fn replica_reconstructed_air_and_forces_never_become_observations() {
        let (e, mut flight) = fixture();
        flight
            .advance(&e, crate::ControlMode::Direct, 1.0 / 120.0)
            .unwrap();
        let local = sample(&flight, &e, Source::LocalAuthority).unwrap();
        let forces = flight.last_forces.as_ref().unwrap();
        assert_eq!(local.air.unwrap().mach, Some(forces.aero.mach));
        assert_eq!(
            local.air.unwrap().dynamic_pressure_pa,
            Some(forces.aero.dynamic_pressure_pa)
        );
        assert_eq!(
            local.loads.unwrap().g_load,
            forces.total_force_inertial_n.length()
                / flight.vehicle.mass_properties.mass_kg
                / 9.80665
        );
        flight.last_forces.as_mut().unwrap().total_force_inertial_n = DVec3::ZERO;
        assert_eq!(
            sample(&flight, &e, Source::LocalAuthority)
                .unwrap()
                .loads
                .unwrap()
                .g_load,
            0.0,
            "free fall has no proper load even in a gravitational field"
        );
        flight.last_forces.as_mut().unwrap().total_force_inertial_n =
            DVec3::Z * flight.vehicle.mass_properties.mass_kg * 9.80665;
        assert!(
            (sample(&flight, &e, Source::LocalAuthority)
                .unwrap()
                .loads
                .unwrap()
                .g_load
                - 1.0)
                .abs()
                < 1e-12
        );
        // Seed precisely the same display-only reconstructions adopt_snapshot
        // installs; source provenance, not mere Some(last_forces), decides.
        let (_, reconstructed) = flight
            .display_loads(
                &e,
                SimTime(flight.flight_time_s),
                crate::ControlMode::Direct,
            )
            .unwrap();
        flight.last_forces = Some(reconstructed);
        flight.reaction_wheel_torque_body_nm = DVec3::new(1.0, 2.0, 3.0);
        let remote = sample(&flight, &e, Source::Replica).unwrap();
        assert!(remote.air.is_none() && remote.loads.is_none() && remote.parachutes.is_none());
        assert_eq!(remote.speed(SpeedFrame::Air), None);
        assert_eq!(
            remote.reaction_wheel_torque_body_nm,
            DVec3::new(1.0, 2.0, 3.0)
        );
        assert_eq!(remote.orbital_speed_mps, local.orbital_speed_mps);
    }

    #[test]
    fn canopy_summary_uses_shared_adoption_validation() {
        let (_, mut flight) = fixture();
        flight.vehicle.parachutes = vec![thessa_sim_core::ParachuteSpec {
            name: "test-canopy".into(),
            reference_area_m2: 20.0,
            drag_coefficient: 1.5,
            reefed_area_fraction: 0.1,
            inflation_time_s: 2.0,
            deploy_pressure_pa: 10000.0,
            max_deploy_dynamic_pressure_pa: 2000.0,
            max_canopy_load_n: 100000.0,
            pack_mass_kg: 12.0,
            repack_time_s: 30.0,
            position_body_m: DVec3::ZERO,
            inertia_body_kg_m2: glam::DMat3::IDENTITY,
            lines: None,
            deformation: None,
        }];
        flight.set_parachutes_armed(false);
        let mut loads = flight.parachute_telemetry().to_vec();
        assert!(!loads.is_empty());
        loads[0].state.phase = ParachutePhase::Deployed;
        loads[0].force_body_n = DVec3::new(3.0, 4.0, 0.0);
        loads[0].dynamic_pressure_pa = 123.0;
        let summary = ParachuteSummary::checked(&flight, &loads).unwrap();
        assert_eq!(summary.open, 1);
        assert_eq!(summary.drag_n, 5.0);
        assert_eq!(summary.dynamic_pressure_pa, 123.0);
        loads[0].force_body_n.x = f64::NAN;
        assert!(ParachuteSummary::checked(&flight, &loads).is_none());
        assert!(ParachuteSummary::checked(&flight, &[]).is_none());
    }

    #[test]
    #[ignore = "CPU cost diagnostic: run release with --ignored --nocapture"]
    fn instrument_readout_target_size_cost() {
        use std::{hint::black_box, time::Instant};
        let (e, _) = fixture();
        let flights: Vec<_> = (0..64)
            .map(|_| FlightAuthority::new(&e, e.body_id("thessa").unwrap()).unwrap())
            .collect();
        for source in [Source::LocalAuthority, Source::Replica] {
            let start = Instant::now();
            for _ in 0..100 {
                for flight in &flights {
                    black_box(sample(flight, &e, source)).unwrap();
                }
            }
            println!(
                "flight_instruments source={source:?} batch=64 batch_us={:.3} craft_us={:.3}; CPU read-model cost, no physics stepping, no game-FPS/speedup claim",
                start.elapsed().as_secs_f64() * 10000.0,
                start.elapsed().as_secs_f64() * 1e6 / 6400.0
            );
        }
    }
}
